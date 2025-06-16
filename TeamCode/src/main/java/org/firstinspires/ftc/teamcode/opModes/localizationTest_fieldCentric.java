package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utility.Gpad;


@TeleOp(name = "LocalizationTest FieldCentric", group = "Tests")
public class localizationTest_fieldCentric extends LinearOpMode {
    Robot robot;
    DriveBase driveBase;
    Gpad g1;
    Gpad g2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap);
        g1 = new Gpad(gamepad1);
        g2 = new Gpad(gamepad2);
        driveBase = new DriveBase(robot, g1, g2);
        driveBase.teleop(true);

        waitForStart();

        while (!isStopRequested()) {
            g1.update();
            g2.update();
            driveBase.update();
            telemetry.addData("X: ", driveBase.getX());
            telemetry.addData("Y: ", driveBase.getY());
            telemetry.addData("H: ", driveBase.getH());

        }
    }
}