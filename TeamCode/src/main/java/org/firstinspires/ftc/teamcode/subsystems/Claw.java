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

    private final Servo servo;
    private final Gpad gp1;
    private final Gpad gp2;

    private State currentState;

    private double target = 0.0;
    private double previousTarget = 0.0;

    private boolean open = false;

    // positions for open and closed states
    private double openPos = 0.0;
    private double closedPos = 1.0;

    private double tolerance = 0.02;

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
        previousTarget = target;
        setState(State.IDLE);
    }

    /** Opens the claw to {@code openPos}. */
    public void open() {
        previousTarget = servo.getPosition();
        target = openPos;
        open = true;
        setState(State.MOVING);
    }

    /** Closes the claw to {@code closedPos}. */
    public void close() {
        previousTarget = servo.getPosition();
        target = closedPos;
        open = false;
        setState(State.MOVING);
    }

    /** Convenience toggle between open and closed positions. */
    public void toggle() {
        if (open) {
            close();
        } else {
            open();
        }
    }

    /** @return true if the servo is near {@code openPos}. */
    public boolean isOpen() {
        return open;
    }

    @Override
    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case MOVING:
                servo.setPosition(target);
                if (Math.abs(servo.getPosition() - target) <= tolerance) {
                    idle();
                }
                break;
            default:
                idle();
                break;
        }
    }

    @Override
    public double getCompletion() {
        // TODO: determine an appropriate completion metric for this subsystem
        return 0.0;
    }
}
