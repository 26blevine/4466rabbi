package org.firstinspires.ftc.teamcode.subsystems.old;

import org.firstinspires.ftc.teamcode.utility.commands.Task;

class LinkageSlideTask extends Task {

    private final LinkageSlide slide;
    private final double targetDistance;

    public LinkageSlideTask(LinkageSlide slide, double targetDistance) {
        this.slide = slide;
        this.targetDistance = targetDistance;
        this.name = "LinkageSlideTask to distance " + targetDistance;
    }

    @Override
    public void start() {
        slide.move(targetDistance);
    }

    @Override
    public void update() {
        slide.update();
    }

    @Override
    public boolean shouldTerminate() {
        return !slide.isMoving();
    }
}
