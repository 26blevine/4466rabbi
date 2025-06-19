package org.firstinspires.ftc.teamcode.utility.commands;


import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

// Represents a single unit of work in the Task Graph
public abstract class Task {
    private List<Task> dependencies = new ArrayList<>(); // Steps that must finish before this one starts
    private boolean started = false; // Has this task started running?
    private boolean finished = false; // Has this task finished running?
    private BooleanSupplier startCondition = () -> true; // Condition to check before starting (default: always true)
    protected String name = "UnnamedTask"; // Optional name for debugging

    // Called once when task starts (example: move motor to a position)
    public abstract void start();

    // Called repeatedly while running (example: keep updating motor movement)
    public abstract void update();

    // Define when the task should terminate (example: motor reached target)
    public abstract boolean shouldTerminate();

    // Connects this task to depend on another task finishing first
    public Task then(Task next) {
        next.dependencies.add(this);
        return next;
    }

    // Set a custom condition for when this task is allowed to start
    public Task startWhen(BooleanSupplier condition) {
        this.startCondition = condition;
        return this;
    }

    // Internal: Can this task start now? (Are all dependencies finished and condition true?)
    boolean canStart() {
        return startCondition.getAsBoolean() && dependencies.stream().allMatch(Task::isTerminated);
    }

    // Internal: Has this task terminated?
    boolean isTerminated() {
        return finished;
    }

    // Internal: Mark the task as started and call start()
    void markStarted() {
        started = true;
        start();
    }

    // Internal: Mark the task as finished
    void markFinished() {
        finished = true;
    }

    // Internal: Has this task started?
    boolean hasStarted() {
        return started;
    }
}
