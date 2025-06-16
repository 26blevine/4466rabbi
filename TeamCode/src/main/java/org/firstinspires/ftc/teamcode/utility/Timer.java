package org.firstinspires.ftc.teamcode.utility;

/**
 * Simple match timer that tracks how much time is left in autonomous or teleop.
 */
public class Timer {
    private static final long AUTO_LENGTH_MS = 30_000; // 30 seconds
    private static final long TELEOP_LENGTH_MS = 120_000; // 120 seconds

    private final long matchLengthMillis;
    private long startTime;
    private boolean running = false;

    /**
     * Creates a new timer.
     *
     * @param isAutonomous true if this timer is for the autonomous period,
     *                     false for teleop
     */
    public Timer(boolean isAutonomous) {
        this.matchLengthMillis = isAutonomous ? AUTO_LENGTH_MS : TELEOP_LENGTH_MS;
    }

    /** Starts the timer. */
    public void start() {
        startTime = System.currentTimeMillis();
        running = true;
    }

    /**
     * Returns the number of seconds that have elapsed since the timer started.
     */
    public double timeElapsed() {
        if (!running) {
            return 0;
        }
        return (System.currentTimeMillis() - startTime) / 1000.0;
    }

    /**
     * Returns the number of seconds remaining in the period.
     */
    public double timeRemaining() {
        if (!running) {
            return matchLengthMillis / 1000.0;
        }
        double remaining = matchLengthMillis - (System.currentTimeMillis() - startTime);
        // Ensure we never return a negative time.
        return Math.max(0, remaining) / 1000.0;
    }

    // Possible future improvements:
    //  - Support pausing and resuming the timer.
    //  - Allow custom match lengths or dynamic adjustment.
    //  - Provide callbacks when time thresholds are reached.
}
