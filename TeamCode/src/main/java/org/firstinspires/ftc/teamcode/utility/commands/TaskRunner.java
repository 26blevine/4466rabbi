package org.firstinspires.ftc.teamcode.utility.commands;


import java.util.ArrayList;
import java.util.List;

// Manages running all tasks
public class TaskRunner {
    private List<Task> activeTasks = new ArrayList<>(); // All tasks currently being processed

    public TaskRunner() {
    }

    // Add the first task to start running
    public void addInitialTask(Task task) {
        activeTasks.add(task);
    }

    // Call this repeatedly (inside OpMode loop)
    public void run() {
        for (Task task : new ArrayList<>(activeTasks)) {
            if (!task.hasStarted() && task.canStart()) { // If the task is ready and hasn't started, start it
                task.markStarted();
            }

            if (task.hasStarted()) { // If the task has started, keep updating it
                task.update();
                if (task.shouldTerminate()) { // If the task terminates, mark it done
                    task.markFinished();
                    activeTasks.remove(task);
                }
            }

        }
    }
}