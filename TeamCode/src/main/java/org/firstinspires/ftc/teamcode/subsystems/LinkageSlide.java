
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
        calcTarget(distance);
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

    public double getTargetDistance(){
        return targetDistance;
    }

    public boolean isRetracted(){
        return currentState == State.IDLE && limitSwitch != null && limitSwitch.isPressed();
    }

    public void calcTarget(double target) {
        //fancy trig
        this.targetPosition = target;
    }

    public double getMaximumExtension() {
        return maximumExtension;
    }

    public void setMaximumExtension(double maximumExtension) {
        this.maximumExtension = maximumExtension;
    }



    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case MOVING:
                servoPair.setPosition(targetPosition);
                //change to idle when done, needs position wire
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
 */
