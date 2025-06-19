package org.firstinspires.ftc.teamcode.utility.commands;


import java.util.List;

// Represents a group of tasks that must all finish before continuing
public class GroupTask extends Task {
    private List<Task> subTasks;

    public GroupTask(List<Task> subTasks) {
        this.subTasks = subTasks;
    }

    @Override
    public void start() {
        for (Task task : subTasks) {
            task.startWhen(() -> true); // No conditions, start immediately
            task.markStarted();
        }
    }

    @Override
    public void update() {
        for (Task task : subTasks) {
            task.update();
        }
    }

    @Override
    public boolean isFinished() {
        for (Task task : subTasks) {
            if (!task.isFinished()) {
                return false;
            }
        }
        return true;
    }
}