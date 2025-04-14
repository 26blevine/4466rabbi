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
}