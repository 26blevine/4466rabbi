package org.firstinspires.ftc.teamcode.utility.commands;

import org.firstinspires.ftc.teamcode.subsystems.old.LinearSlide_Old;

public class LinearSlideTask extends Task {

    private final LinearSlide_Old slide;
    private final int target;

    public LinearSlideTask(LinearSlide_Old slide, int target) {
        this.slide = slide;
        this.target = target;
        this.name = "LinearSlideTask to position " + target;
    }

    @Override
    public void start() {
        slide.move(target);
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
