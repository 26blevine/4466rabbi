package org.firstinspires.ftc.teamcode.utility.commands;

import org.firstinspires.ftc.teamcode.subsystems.old.LinkageSlide;

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
    public boolean isFinished() {
        return !slide.isMoving();
    }
}
