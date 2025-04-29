package org.firstinspires.ftc.teamcode.utility.commands;

import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.utility.Task;

class DriveBaseTask extends Task {

    private final DriveBase driveBase;
    private final Pose target;
    private final boolean holdOnEnd;

    public DriveBaseTask(DriveBase driveBase, Pose target, boolean holdOnEnd) {
        this.driveBase = driveBase;
        this.target = target;
        this.holdOnEnd = holdOnEnd;
        this.name = "DriveBaseTask to pose with hold=" + holdOnEnd;
    }

    @Override
    public void start() {
        driveBase.autonomous(target, holdOnEnd);
    }

    @Override
    public void update() {
        driveBase.update();
    }

    @Override
    public boolean isFinished() {
        return !driveBase.isFollowing();
    }
}
