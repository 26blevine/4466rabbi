// --- New Linear Servo Slide Subsystem ---
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.utility.ServoPair;
import org.firstinspires.ftc.teamcode.utility.GamepadPair;

public class LinkageSlide {

    private enum State {
        IDLE,
        MOVING
    }

    private State currentState;

    private final ServoPair servoPair;
    private final GamepadPair gp;
    private final TouchSensor limitSwitch;

    private double targetPosition = 0.0;
    private double targetDistance = 0.0;
    private double maximumExtension = 1.0; // Maximum extension distance, scale 0-1 for now
    private double tolerance = 0.01; // Tolerance for position checking
    private double manualK = 0.005; // Manual jog increment factor

    // Constants for linkage calculations
    private static final double vertOffset = 2.0;
    private static final double linkageLength = 10.0;
    private static final double M = -1.0 / 90.0;
    private static final double B = 1.0;

    public LinkageSlide(ServoPair servoPair, GamepadPair gp, TouchSensor limitSwitch) {
        this.servoPair = servoPair;
        this.gp = gp;
        this.limitSwitch = limitSwitch;
        this.currentState = State.IDLE;
    }

    private void setState(State newState) {
        this.currentState = newState;
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void move(double distance) {
        distance = Math.max(0.0, Math.min(distance, maximumExtension)); // clamp distance
        targetDistance = distance;
        targetPosition = calcTargetPosition(targetDistance);
        setState(State.MOVING);
    }

    public boolean isIdle() {
        return currentState == State.IDLE;
    }

    public boolean isMoving() {
        return currentState == State.MOVING;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public boolean isRetracted() {
        return currentState == State.IDLE && limitSwitch != null && limitSwitch.isPressed();
    }

    public void calcTarget(double target) {
        this.targetPosition = calcTargetPosition(target);
    }

    private double calcTargetPosition(double target) {
        double a = Math.sqrt(target * target + vertOffset * vertOffset);
        double b = 2 * linkageLength;
        double c = Math.acos(a / b);
        double d = M * c + B;
        return d;
    }

    public double getMaximumExtension() {
        return maximumExtension;
    }

    public void setMaximumExtension(double maximumExtension) {
        this.maximumExtension = maximumExtension;
    }

    public double getManualK() {
        return manualK;
    }

    public void setManualK(double manualK) {
        this.manualK = manualK;
    }

    private void manualMove(){
        double r = gp.getTrigger(1, "right_trigger");
        double l = gp.getTrigger(1, "left_trigger");
        double delta = r - l; // Positive -> extend, Negative -> retract

        if (Math.abs(delta) > 0.1) {
            double increment = delta * manualK;
            targetDistance = Math.max(0.0, Math.min(targetDistance + increment, maximumExtension));
            targetPosition = calcTargetPosition(targetDistance);
            setState(State.MOVING);
        }
    }

    public void update() {

        manualMove();
        switch (currentState) {
            case IDLE:
                break;
            case MOVING:
                servoPair.setPosition(targetPosition);
                if (servoPair.isAtPosition(targetPosition, tolerance)) {
                    idle();
                }
                break;
            default:
                idle();
                break;
        }
    }
}

/*
 * LinearServoSlide Notes and Future Ideas:
 * - Proportional control only for now. May upgrade to PID if overshoot or instability occurs.
 * - Move now clamps input distance between 0 and maximumExtension for safety.
 * - Limit switch is optional; if provided, used for basic homing/safety checks.
 * - In the future: add hard stops based on limit switch triggers.
 * - Allow resetting servo zero if limit is hit.
 * - ManualK, Precision, MaximumExtension could be adjustable via dashboard.
 * - Could later integrate soft start/stop ramps to prevent mechanical jerks.
 * - Added tolerance for smarter idling and separated calculation logic for flexibility.
 * - Updated position checking to use ServoPair.isAtPosition() for cleaner logic.
 * - Renamed calculateTarget to calcTargetPosition.
 * - Added manual slide jogging with right/left triggers.
 */
