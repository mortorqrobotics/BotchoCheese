package frc.BotchoCheese.Utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class DebugLog {
    // Keep false for clean match logs. Flip true when actively debugging.
    public static final boolean DEBUG = false;

    private static final Map<String, Double> lastWarningTimeByKey = new HashMap<>();

    private DebugLog() {}

    public static void info(String message) {
        DriverStation.reportWarning("[INFO] " + message, false);
    }

    public static void debug(String message) {
        if (DEBUG) {
            DriverStation.reportWarning("[DEBUG] " + message, false);
        }
    }

    public static void warn(String message) {
        DriverStation.reportWarning(message, false);
    }

    public static void warnThrottled(String key, String message, double minIntervalSeconds) {
        double now = Timer.getFPGATimestamp();
        double lastTime = lastWarningTimeByKey.getOrDefault(key, Double.NEGATIVE_INFINITY);
        if (now - lastTime >= minIntervalSeconds) {
            lastWarningTimeByKey.put(key, now);
            DriverStation.reportWarning(message, false);
        }
    }

    public static void error(String message, StackTraceElement[] stackTrace) {
        DriverStation.reportError(message, stackTrace);
    }
}
