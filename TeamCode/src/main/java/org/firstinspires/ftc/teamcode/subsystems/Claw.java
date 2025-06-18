package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utility.Gpad;

/**
 * Simple claw subsystem using a single servo.
 * Takes two {@link Gpad} inputs for control.
 */
public class Claw implements Subsystem {

    private enum State {
        IDLE,
        MOVING
    }

    private State currentState;

    private final Gpad gp1;
    private final Gpad gp2;
    private final Servo servo;

    // positions for open and closed states
    private double openPos = 0.0;
    private double closedPos = 1.0;

    // completion tracking (not yet implemented)
    private double completion = 0.0;

    public Claw(Servo servo, Gpad gp1, Gpad gp2) {
        this.servo = servo;
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.currentState = State.IDLE;
    }

    private void setState(State state) {
        this.currentState = state;
    }

    /** Puts the subsystem in the idle state. */
    public void idle() {
        setState(State.IDLE);
    }

    /** Opens the claw to {@code openPos}. */
    public void open() {
        servo.setPosition(openPos);
        setState(State.MOVING);
    }

    /** Closes the claw to {@code closedPos}. */
    public void close() {
        servo.setPosition(closedPos);
        setState(State.MOVING);
    }

    /** Convenience toggle between open and closed positions. */
    public void toggle() {
        if (isOpen()) {
            close();
        } else {
            open();
        }
    }

    /** @return true if servo is near {@code openPos}. */
    public boolean isOpen() {
        return Math.abs(servo.getPosition() - openPos) < 0.05;
    }

    public void setOpenPos(double pos) { this.openPos = pos; }
    public void setClosedPos(double pos) { this.closedPos = pos; }
    public double getOpenPos() { return openPos; }
    public double getClosedPos() { return closedPos; }

    @Override
    public void update() {
        // Basic control example: gp1.b closes, gp1.a opens
        if (gp1.a || gp2.a) {
            open();
        } else if (gp1.b || gp2.b) {
            close();
        }

        switch (currentState) {
            case MOVING:
                // Servo movement is instantaneous; add sensor checks here if needed
                idle();
                break;
            case IDLE:
                break;
            default:
                open();
                break;
        }
    }

    @Override
    public double getCompletion() {
        // TODO: implement completion tracking
        return completion;
    }
}
