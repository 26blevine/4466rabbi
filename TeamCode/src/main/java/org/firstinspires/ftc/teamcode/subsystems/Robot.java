package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
public class Robot {

    public static double X_OFFSET = 24;
    public static double Y_OFFSET = -166.05;

    public DcMotor slideMotorLeft, slideMotorRight, frontLeft, backLeft, frontRight, backRight;
    public HardwareMap hardwareMap;
    public GoBildaPinpointDriver pinpointDriver;
    public Follower follower;

    public Robot(HardwareMap map) {

        this.hardwareMap = map;

        this.slideMotorLeft = map.get(DcMotor.class, "slideMotorLeft");
        this.slideMotorRight = map.get(DcMotor.class, "slideMotorRight");

        this.frontLeft = map.get(DcMotor.class, "frontLeft");
        this.backLeft = map.get(DcMotor.class, "backLeft");
        this.frontRight = map.get(DcMotor.class, "frontRight");
        this.backRight = map.get(DcMotor.class, "backRight");

        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        try {
            pinpointDriver = getDevice(GoBildaPinpointDriver.class, "odo");
            pinpointDriver.setOffsets(X_OFFSET, Y_OFFSET);
            pinpointDriver.resetPosAndIMU();
            follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        } catch (Exception e) {
        }
    }


    //If robot breaks, it was this
    public void updatePinpoint() {
        if (pinpointDriver == null) {
            return;
        }
        pinpointDriver.update();
    }

    public Pose getPoseEstimate() {
        if (pinpointDriver == null) {
            return new Pose(0, 0, 0);
        }

        Pose pos = pinpointDriver.getPosition();
        double x = pos.getX();
        double y = pos.getY();
        double heading = pos.getHeading();
        return new Pose(x, y, heading);
    }

    public void setPoseEstimate(Pose input) {
        if (pinpointDriver == null) {
            return;
        }
        pinpointDriver.setPosition(input);
    }

    public <T> T getDevice(Class<? extends T> classOfDevice, String deviceName) {
        T toReturn = null;
        try {
            toReturn = hardwareMap.get(classOfDevice, deviceName);
        } catch (Exception e) {
        }
        return toReturn;
    }
}


