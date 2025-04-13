package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.localization.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.teamcode.utility.GamepadPair;

public class DriveBase {

    private enum State {
        IDLE,
        HOLD,
        AUTONOMOUS,
        TELEOP
    }


    //private objects
    private State currentState;
    private Robot robot;
    private GamepadPair gamepadPair;
    private boolean fieldCentric, holdPathEnd;
    private Path autoPath;
    private Follower follower;

    //setters + getters
    private void setState(State s) {
        currentState = s;
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void hold() {
        setState(State.HOLD);
    }

    public void getTeleopMovement() {
        double x = gamepadPair.joystickValue(1, "left", "x");
        double y = gamepadPair.joystickValue(1, "left", "y");
        double heading = gamepadPair.joystickValue(1, "right", "x");
        follower.setTeleOpMovementVectors(x, y, heading, !fieldCentric);
    }

    public void teleop(boolean f) {

        setFieldCentric(f);
        getTeleopMovement();
        follower.setStartingPose(robot.getPoseEstimate());
        setState(State.TELEOP);
    }

    public void autonomous(Pose target, boolean holdOnEnd) {
        holdPathEnd = holdOnEnd;
        Pose currentPose = robot.getPoseEstimate();
        autoPath = new Path(new BezierCurve(new Point(currentPose), new Point(target)));
        autoPath.setLinearHeadingInterpolation(currentPose.getHeading(), target.getHeading());
        setState(State.AUTONOMOUS);

    }

    public State getCurrentState() {
        return currentState;
    }

    public Pose getPos(){
        return follower.getPose();
    }

    public boolean isMoving() {
        return (currentState == State.AUTONOMOUS || currentState == State.TELEOP);
    }

    public boolean isHolding() {
        return currentState == State.HOLD;
    }

    public boolean isIdle() {
        return currentState == State.IDLE;
    }

    public void setFieldCentric(boolean f) {
        this.fieldCentric = f;
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }


    public DriveBase(Robot r, GamepadPair gp) {
        this.robot = r;
        this.follower = robot.follower;
        this.gamepadPair = gp;
        this.currentState = State.IDLE;
    }


    public void update() {

        follower.update();

        switch (currentState) {
            case IDLE:
                // does nothing
                break;
            case HOLD:
                follower.holdPoint(robot.getPoseEstimate());
                break;
            case AUTONOMOUS:
                follower.followPath(autoPath, holdPathEnd);
                if (!follower.isBusy()) {
                    if (holdPathEnd){
                        hold();
                    } else {
                        idle();
                    }
                }
                break;
            case TELEOP:
                follower.startTeleopDrive();
                break;
            default:
                // just in case we get an invalid state
                break;
        }
    }

}
