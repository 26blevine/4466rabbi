package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.pedropathing.util.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.utility.Gpad;

/**
 * Controls a dual motor linear slide using a PIDF controller. The slide can
 * move to target positions, hold its current position or be controlled
 * manually. Encoders are reset automatically when the bottom magnet switch is
 * triggered.
 */
public class LinearSlide_Test implements Subsystem {

    /** Operating states for the slide. */
    private enum State { IDLE, MOVING, HOLD, RESET }

    /*-----------------------------------------------------------------------*/
    /* Hardware references                                                   */
    /*-----------------------------------------------------------------------*/
    private final DcMotor leftMotor;
    private final DcMotor rightMotor;
    private final Gpad gp1;
    private final Gpad gp2;
    private final TouchSensor magnetSwitch;

    /*-----------------------------------------------------------------------*/
    /* Configuration fields                                                  */
    /*-----------------------------------------------------------------------*/
    private PIDFController pidfController;
    private int precision = 0;
    private double manualRatio = 0.0;
    private int maxHeight = 0;
    private int safetyDistance = 0;

    /*-----------------------------------------------------------------------*/
    /* Runtime state                                                         */
    /*-----------------------------------------------------------------------*/
    private State currentState;
    private double speed = 0.0;
    private boolean zeroed = false;
    private int currentPos = 0;
    private int targetPos = 0;

    /**
     * Constructs the linear slide subsystem.
     *
     * @param leftMotor    left slide motor
     * @param rightMotor   right slide motor
     * @param gp1          primary gamepad wrapper
     * @param gp2          secondary gamepad wrapper
     * @param magnetSwitch magnetic limit switch at the slide bottom
     */
    public LinearSlide_Test(DcMotor leftMotor, DcMotor rightMotor,
                            Gpad gp1, Gpad gp2, TouchSensor magnetSwitch) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.gp1 = gp1;
        this.gp2 = gp2;
        this.magnetSwitch = magnetSwitch;

        CustomPIDFCoefficients coeffs = new CustomPIDFCoefficients(1.0, 0.0, 0.1, 0.0);
        this.pidfController = new PIDFController(coeffs);

        restartMotors();
        currentState = State.IDLE;
    }

    /*-----------------------------------------------------------------------*/
    /* Accessors                                                             */
    /*-----------------------------------------------------------------------*/
    /** @return current slide encoder position */
    public int getCurrentPos() {
        return currentPos;
    }

    /** @return movement precision in ticks */
    public int getPrecision() {
        return precision;
    }

    /** Sets the tolerance for reaching a target in ticks. */
    public void setPrecision(int precision) {
        this.precision = precision;
    }

    /** @return target position in ticks */
    public int getTargetPos() {
        return targetPos;
    }

    /**
     * Sets the target position clamped between 0 and {@code maxHeight - safetyDistance}.
     */
    public void setTargetPos(int targetPos) {
        int max = Math.max(0, maxHeight - safetyDistance);
        int clamped = Math.max(0, Math.min(targetPos, max));
        this.targetPos = clamped;
    }

    /** @return manual movement ratio */
    public double getManualRatio() {
        return manualRatio;
    }

    /** Sets the scaling factor for manual control. */
    public void setManualRatio(double manualRatio) {
        this.manualRatio = manualRatio;
    }

    /** @return maximum allowed height in ticks */
    public int getMaxHeight() {
        return maxHeight;
    }

    /** Sets the maximum allowed height for the slide. */
    public void setMaxHeight(int maxHeight) {
        this.maxHeight = Math.max(0, maxHeight);
    }

    /** @return safety distance from the top in ticks */
    public int getSafetyDistance() {
        return safetyDistance;
    }

    /** Sets the safety distance from the maximum height. */
    public void setSafetyDistance(int safetyDistance) {
        this.safetyDistance = Math.max(0, Math.min(safetyDistance, maxHeight));
    }

    /** @return current PIDF controller */
    public PIDFController getPidfController() {
        return pidfController;
    }

    /** Replaces the PIDF controller. */
    public void setPidfController(PIDFController controller) {
        this.pidfController = controller;
    }

    /** @return last set motor speed */
    public double getSpeed() {
        return speed;
    }

    /*-----------------------------------------------------------------------*/
    /* State transition helpers                                              */
    /*-----------------------------------------------------------------------*/
    /** Puts the slide in the idle state. */
    public void idle() {
        setState(State.IDLE);
    }

    /** Begins moving to {@code target}. */
    public void move(int target) {
        setTargetPos(target);
        setState(State.MOVING);
    }

    /** Holds the current target position. */
    public void hold() {
        setState(State.HOLD);
    }

    /** Starts the reset routine. */
    public void reset() {
        setState(State.RESET);
    }

    /**
     * Call periodically to update motor outputs based on the current state.
     */
    public void update() {
        currentPos = leftMotor.getCurrentPosition();

        switch (currentState) {
            case IDLE:
                setSpeed(0);
                break;
            case MOVING:
                moveTo();
                break;
            case HOLD:
                holdPos();
                break;
            case RESET:
                zero();
                break;
            default:
                idle();
                break;
        }
    }

    /*-----------------------------------------------------------------------*/
    /* Private helpers                                                       */
    /*-----------------------------------------------------------------------*/
    private void setState(State state) {
        this.currentState = state;
    }

    /** Sets both motors to a clamped speed and records it. */
    private void setSpeed(double value) {
        double clamped = Math.max(-1.0, Math.min(1.0, value));
        leftMotor.setPower(clamped);
        rightMotor.setPower(clamped);
        this.speed = clamped;
    }

    /** Resets encoders and puts motors back into run-using-encoder mode. */
    private void restartMotors() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** @return {@code true} if the bottom limit switch is pressed. */
    private boolean isMagnetTriggered() {
        return magnetSwitch != null && magnetSwitch.isPressed();
    }

    /**
     * Drives the slide towards zero until the magnet switch is triggered.
     * Once triggered, the encoders are reset and the slide idles.
     */
    private void zero() {
        if (!isMagnetTriggered()) {
            zeroed = false;
            setTargetPos(0);
            moveTo();
            return;
        }

        if (!zeroed) {
            setSpeed(0);
            restartMotors();
            currentPos = 0;
            targetPos = 0;
            zeroed = true;
        }
        idle();
    }

    /** Moves toward {@code targetPos} using the PIDF controller. */
    private void moveTo() {
        pidfController.setTargetPosition(targetPos);
        pidfController.updatePosition(currentPos);
        double output = pidfController.runPIDF();
        setSpeed(output);

        if (Math.abs(targetPos - currentPos) < precision) {
            hold();
        }
        // TODO: handle additional actions when the target is reached
    }

    /**
     * Handles manual slide movement based on gamepad triggers. When the magnet
     * switch is pressed, the slide zeros itself via {@link #zero()} and
     * downward movement is prevented.
     */
    private void manualMove() {
        double output = (gp1.RTrigger - gp1.LTrigger) * manualRatio;

        if (isMagnetTriggered()) {
            if (!zeroed) {
                zero();
            }
            if (output < 0) {
                output = 0;
            }
        } else {
            zeroed = false;
        }

        setSpeed(output);
    }

    /** Maintains the target position using PIDF. */
    private void holdPos() {
        pidfController.setTargetPosition(targetPos);
        pidfController.updatePosition(currentPos);
        double output = pidfController.runPIDF();
        setSpeed(output);
    }

    @Override
    public double getCompletion() {
        // TODO: Determine an appropriate completion metric for this subsystem.
        // For now, we simply return 0.
        return 0;
    }
}

/*
 * Future considerations:
 * - Expose manualMove through its own state if operator control is needed.
 * - Tune PIDF coefficients to match the real hardware response.
 * - Provide runtime configuration via the FTC dashboard.
 */
