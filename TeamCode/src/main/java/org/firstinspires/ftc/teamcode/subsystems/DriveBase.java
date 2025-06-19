package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.utility.Gpad;

/**
 * Drive subsystem responsible for robot locomotion.
 */
public class DriveBase implements Subsystem {

    private enum State {
        IDLE,
        HOLD,
        AUTONOMOUS,
        TELEOP
    }


    //private objects
    private State currentState;
    private Robot robot;
    private Gpad gp1;
    private Gpad gp2;
    private boolean fieldCentric, holdPathEnd;
    private Path autoPath;
    private Follower follower;
    private Pose currentPose;

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

    private void getTeleopMovement() {
        double x = -gp1.stickLY;
        double y = -gp1.stickLX;
        double heading = -gp1.stickRX;
        follower.setTeleOpMovementVectors(x, y, heading, !fieldCentric);
    }

    public void teleop(boolean f) {
        setFieldCentric(f);
        follower.setStartingPose(currentPose);
        follower.startTeleopDrive();
        setState(State.TELEOP);
    }

    public void autonomous(Pose target, boolean holdOnEnd) {
        holdPathEnd = holdOnEnd;
        Pose startPose = currentPose;
        autoPath = new Path(new BezierCurve(new Point(startPose), new Point(target)));
        autoPath.setLinearHeadingInterpolation(startPose.getHeading(), target.getHeading());
        setState(State.AUTONOMOUS);
    }

    /**
     * Starts an autonomous drive using a precomputed path.
     *
     * @param path       the path to follow
     * @param holdOnEnd  whether to hold position when the path completes
     */
    public void autonomous(Path path, boolean holdOnEnd) {
        holdPathEnd = holdOnEnd;
        autoPath = path;
        setState(State.AUTONOMOUS);
    }

    public State getCurrentState() {
        return currentState;
    }

    public Pose getPos(){
        return currentPose;
    }

    public double getX() {
        return currentPose.getX();
    }

    public double getY() {
        return currentPose.getY();
    }

    public double getH() {
        return currentPose.getHeading();
    }

    public boolean isFollowing(){
        return follower.isBusy();
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


    public DriveBase(Robot r, Gpad g1, Gpad g2) {
        this.robot = r;
        this.follower = new Follower(robot.hardwareMap, FConstants.class, LConstants.class);
        this.gp1 = g1;
        this.gp2 = g2;
        this.currentState = State.IDLE;
        this.currentPose = follower.getPose();
        this.fieldCentric = false;
    }


    @Override
    public void update() {

        follower.update();
        currentPose = follower.getPose();

        switch (currentState) {
            case IDLE:
                // does nothing
                break;
            case HOLD:
                follower.holdPoint(currentPose);
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
                getTeleopMovement();
                break;
            default:
                // just in case we get an invalid state
                break;
        }
    }

    /**
     * Returns the completion percentage of the current action.
     *
     * TODO: Determine how to calculate this based on follower/path state.
     */
    @Override
    public double getCompletion() {
        // TODO: implement completion tracking
        return 0;
    }

}
