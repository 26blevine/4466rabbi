package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.utility.ServoPair;
import org.firstinspires.ftc.teamcode.utility.Gpad;

/**
 * Basic arm subsystem using a pair of servos.
 */
public class Arm implements Subsystem {

    private enum State {
        IDLE,
        MOVING
    }

    private final ServoPair servos;
    private final Gpad gp1;
    private final Gpad gp2;

    private State currentState;

    private double target = 0.0;
    private double previousTarget = 0.0;

    private final double intakePos = 0.25;
    private final double outtakePos = 0.75;
    private double tolerance = 0.02;

    public Arm(Gpad gp1, Gpad gp2, ServoPair servos) {
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.servos = servos;
        this.currentState = State.IDLE;
    }

    private void setState(State state) {
        this.currentState = state;
    }

    public void idle() {
        previousTarget = target;
        setState(State.IDLE);
    }

    public void intake() {
        target = intakePos;
        setState(State.MOVING);
    }

    public void outtake() {
        target = outtakePos;
        setState(State.MOVING);
    }

    @Override
    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case MOVING:
                servos.setPosition(target);
                if (servos.isAtPosition(target, tolerance)) {
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
        // TODO: implement completion tracking
        return 0;
    }
}

/*
 * Future ideas and improvements:
 * - Expose setters for intake/outtake positions via dashboard for quick tuning.
 * - Allow runtime adjustment of tolerance for precision vs. speed trade-offs.
 * - Add manual overrides using the provided Gpads for direct driver control.
 */
