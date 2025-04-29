package org.firstinspires.ftc.teamcode.utility.commands;

import org.firstinspires.ftc.teamcode.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.utility.Task;

public class LinearSlideTask extends Task {

    private final LinearSlide slide;
    private final int target;

    public LinearSlideTask(LinearSlide slide, int target) {
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
