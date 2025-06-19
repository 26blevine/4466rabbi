package org.firstinspires.ftc.teamcode.utility.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashSet;
import java.util.Set;

// Centralized telemetry manager to control and filter logs
class TelemetryManager {
    private Set<String> enabledTags = new HashSet<>(); // Tags that are allowed
    private boolean verbose = false; // If true, log everything
    private Telemetry telemetry;

    public TelemetryManager(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void enableTag(String tag) {
        enabledTags.add(tag);
    }

    public void disableTag(String tag) {
        enabledTags.remove(tag);
    }

    public void setVerbose(boolean verbose) {
        this.verbose = verbose;
    }

    public void log(String tag, String message) {
        if (verbose || enabledTags.contains(tag)) {
            telemetry.addData(tag, message);
        }
    }

    public void update() {
        telemetry.update();
    }
}