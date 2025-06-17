package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.PIDFController;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.GamepadPair;

public class LinearSlide {

    private enum State {
        IDLE,
        MOVING,
        HOLD
    }

    private State currentState;

    private final DcMotor motorL;
    private final DcMotor motorR;
    private final PIDFController pidfController;
    private final GamepadPair gp;
    private final TouchSensor magnetSwitch;

    private int precision = 10;
    private int currentPos = 0;
    private int targetPos = 0;
    private int maxHeight = 0;
    private int safetyDistance = 10;

    // Track the previous state of the magnet switch so we only reset once per press
    private boolean wasMagnetPressed = false;

    // K multiplier for manual adjustments
    private double manualRatio = 10.0; // default can be changed as desired

    public LinearSlide(DcMotor motorL, DcMotor motorR, GamepadPair gp, TouchSensor magnetSwitch) {
        this.motorL = motorL;
        this.motorR = motorR;
        this.gp = gp;
        this.magnetSwitch = magnetSwitch;
        this.currentState = State.IDLE;

        // Example PIDF coefficients: tune these later
        CustomPIDFCoefficients coeffs = new CustomPIDFCoefficients(1.0, 0.0, 0.1, 0.0);
        pidfController = new PIDFController(coeffs);
    }

    public double getManualRatio() {
        return manualRatio;
    }

    public void setManualRatio(double k) {
        this.manualRatio = k;
    }

    private void setState(State newState) {
        this.currentState = newState;
    }

    public void idle() {
        setState(State.IDLE);
    }

    public void move(int t) {
        setTarget(t);
        setState(State.MOVING);
    }

    public int getPos(){
        return this.currentPos;
    }

    public void hold() {
        setState(State.HOLD);
    }

    public boolean isIdle() {
        return currentState == State.IDLE;
    }

    public boolean isMoving() {
        return currentState == State.MOVING;
    }

    public boolean isHold() {
        return currentState == State.HOLD;
    }

    // Instead of returning the sensor object, return whether it's pressed
    // If magnetSwitch is null, we can return false
    public boolean getMagnetSwitch() {
        if (magnetSwitch == null) {
            return false;
        }
        return magnetSwitch.isPressed();
    }

    public int getTarget() {
        return targetPos;
    }

    public void setTarget(int target) {
        // Clamp the requested target within the valid range of the slide
        this.targetPos = Math.max(0, Math.min(target, maxHeight));
    }

    public int getMaxHeight() {
        return maxHeight;
    }

    public void setMaxHeight(int maxHeight) {
        this.maxHeight = maxHeight;
    }

    public int getSafetyDistance() {
        return safetyDistance;
    }

    public void setSafetyDistance(int safetyDistance) {
        this.safetyDistance = safetyDistance;
    }

    // Set both motors to the same power value, clamping to [-1, 1].
    private void setPowers(double p) {
        double clipped = Math.max(-1.0, Math.min(1.0, p));
        motorL.setPower(clipped);
        motorR.setPower(clipped);
    }

    private void manualMove(){
        double r = gp.getTrigger(1, "right_trigger");
        double l = gp.getTrigger(1, "left_trigger");
        double delta = r - l; // positive -> up, negative -> down
        // Only override if abs(delta) > 0.1
        if (Math.abs(delta) > 0.1) {
            // Increment the target by delta * manualRatio so repeated input gradually adjusts
            // the desired position while the PID controller handles movement.
            int increment = (int)(delta * manualRatio);
            setTarget(getTarget() + increment);
            setState(State.MOVING);
        }
    }

    private void moveTo() {
        // Prevent driving upward past the top limit but still allow downward motion
        if (currentPos >= maxHeight - safetyDistance && targetPos > currentPos) {
            targetPos = maxHeight;
            hold();
            return;
        }

        // 2) PID control to current target
        pidfController.setTargetPosition(targetPos);
        pidfController.updatePosition(currentPos);
        double moveOutput = pidfController.runPIDF();
        setPowers(moveOutput);

        // 3) If we’re close, we do the normal thing (switch to hold)
        if (Math.abs(pidfController.getError()) < precision) {
            hold();
        }
    }

    private void holdPos() {
        pidfController.setTargetPosition(currentPos);
        pidfController.updatePosition(currentPos);
        double holdOutput = pidfController.runPIDF();
        setPowers(holdOutput);
    }

    // private reset method
    private void reset(boolean magnetPressed) {
        boolean shouldReset = false;
        if (magnetSwitch != null) {
            if (magnetPressed && !wasMagnetPressed) {
                shouldReset = true;
            } else if (currentPos <= 0) {
                shouldReset = true;
            }

            if (shouldReset) {
                // stop motors before reconfiguring
                setPowers(0);

                // reset encoders
                motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // immediately switch back so PID/teleop can use encoders
                // (change to RUN_WITHOUT_ENCODER if your design requires it)
                motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // reset bookkeeping and hold at zero
                currentPos = 0;
                targetPos = 0;
                hold();
            }
        }
        // remember current switch state for next loop
        wasMagnetPressed = magnetPressed;
    }

    public void update() {
        currentPos = motorL.getCurrentPosition();
        boolean magnetPressed = magnetSwitch != null && magnetSwitch.isPressed();
        reset(magnetPressed);
        manualMove();
        switch (currentState) {
            case IDLE:
                setPowers(0);
                break;

            case MOVING:
                moveTo();
                break;

            case HOLD:
                holdPos();
                break;

            default:
                // fallback
                idle();
                break;
        }
    }
}

/*
Additional Information:

- The manualRatio variable (defaults to 10.0) scales how many encoder ticks are added per loop.
- In moveTo(), we read the difference between Right Trigger (up) and Left Trigger (down). If abs(delta) > 0.1,
  we modify the current target by adding an increment = (delta * manualRatio). This lets the driver nudge the slide
  while the PID controller handles smooth motion.
- The rest of the moveTo() logic remains, so the PID aims for the new target.
- This approach allows partial pressing of triggers to raise or lower the slide gradually.
- If no triggers are pressed, the slide just continues to move to the previously set target.
- All other states remain the same (IDLE, HOLD).
*/
