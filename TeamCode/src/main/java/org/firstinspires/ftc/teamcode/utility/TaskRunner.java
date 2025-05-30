package org.firstinspires.ftc.teamcode.utility;


import java.util.ArrayList;
import java.util.List;

// Manages running all tasks
public class TaskRunner {
    private List<Task> activeTasks = new ArrayList<>(); // All tasks currently being processed
    private TelemetryManager telemetryManager;

    public TaskRunner(TelemetryManager telemetryManager) {
        this.telemetryManager = telemetryManager;
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
                if (task.isFinished()) { // If the task finishes, mark it done
                    task.markFinished();
                    activeTasks.remove(task);
                }
            }

            task.toTelemetry(telemetryManager); // Collect telemetry from active tasks
        }
        telemetryManager.update(); // Push telemetry to driver station once per loop
    }
}