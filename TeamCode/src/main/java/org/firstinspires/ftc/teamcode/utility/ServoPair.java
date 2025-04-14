package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoPair {

    private final Servo servoA;
    private final Servo servoB;
    private boolean useAForPosition = true; // Flag to decide which servo to read position from

    public ServoPair(Servo servoA, Servo servoB) {
        this.servoA = servoA;
        this.servoB = servoB;
    }

    public void setPosition(double position) {
        servoA.setPosition(position);
        servoB.setPosition(position);
    }

    public double getPosition() {
        return useAForPosition ? servoA.getPosition() : servoB.getPosition();
    }

    public void setUseAForPosition(boolean useA) {
        this.useAForPosition = useA;
    }

    public boolean isUsingA() {
        return useAForPosition;
    }

    /**
     * Returns true if the servo position is within the tolerance of the target.
     * @param target target position to check against.
     * @param tolerance acceptable range around the target.
     * @return true if current position is within [target - tolerance, target + tolerance]
     */
    public boolean isAtPosition(double target, double tolerance) {
        return Math.abs(getPosition() - target) <= tolerance;
    }
}
