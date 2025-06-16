package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.utility.GamepadPair;


@TeleOp(name = "LocalizationTestBSL 2", group = "Tests")
public class localizationTest_bsl extends LinearOpMode {
    Robot robot;
    DriveBase driveBase;
    GamepadPair gp;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap);
        gp = new GamepadPair(gamepad1, gamepad2);
        driveBase = new DriveBase(robot, gp);
        driveBase.teleop(false);

        waitForStart();

        while (!isStopRequested()) {
            driveBase.update();
            telemetry.addData("X: ", driveBase.getPos().getX());
            telemetry.addData("Y: ", driveBase.getPos().getY());
            telemetry.addData("H: ", driveBase.getPos().getHeading());

        }
    }
}