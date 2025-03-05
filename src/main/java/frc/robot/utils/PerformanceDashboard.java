// src/main/java/frc/robot/utils/PerformanceDashboard.java
package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

/**
 * PerformanceDashboard - THE ULTIMATE PERFORMANCE TRACKER!
 * 
 * This utility helps us optimize our code by monitoring execution times,
 * CPU usage, loop frequency, and other critical performance metrics.
 * 
 * HOW TO USE:
 * - Call startTimer("name") at the beginning of a section
 * - Call stopTimer("name") at the end
 * - The dashboard will show timing metrics
 * 
 * This has LITERALLY saved us HOURS of debugging time!
 * 
 * coded by paysean - March 2025
 */
public class PerformanceDashboard {
    
    // ===== TIMING DATA =====
    private static Map<String, TimingData> timers = new HashMap<>();
    private static double loopStartTime = 0.0;
    private static int loopCounter = 0;
    private static double lastLoopTime = 0.0;
    private static double maxLoopTime = 0.0;
    private static double avgLoopTime = 0.0;
    private static double batteryStartVoltage = 12.0;
    
    // ===== DISPLAY STATE =====
    private static boolean verboseMode = false;
    private static int updateCounter = 0;
    
    /**
     * Class to track timing metrics for each monitored section
     */
    private static class TimingData {
        String name;
        double startTime;
        double lastDuration;
        double totalDuration = 0.0;
        double minDuration = Double.MAX_VALUE;
        double maxDuration = 0.0;
        int callCount = 0;
        
        public TimingData(String name) {
            this.name = name;
        }
    }
    
    /**
     * Initialize the performance dashboard
     * 
     * @param verbose Whether to show detailed metrics
     */
    public static void initialize(boolean verbose) {
        verboseMode = verbose;
        loopStartTime = Timer.getFPGATimestamp();
        batteryStartVoltage = RobotController.getBatteryVoltage();
        
        SmartDashboard.putBoolean("Performance_Tracking", true);
        
        System.out.println("");
        System.out.println(">> PERFORMANCE DASHBOARD ACTIVATED! <<");
        System.out.println(">> VERBOSE MODE: " + (verbose ? "ENABLED" : "DISABLED") + " <<");
        System.out.println("");
    }
    
    /**
     * Start a timer for a specific code section
     * 
     * @param name Name of the code section to time
     */
    public static void startTimer(String name) {
        if (!timers.containsKey(name)) {
            timers.put(name, new TimingData(name));
        }
        
        TimingData timer = timers.get(name);
        timer.startTime = Timer.getFPGATimestamp();
    }
    
    /**
     * Stop a timer and record metrics
     * 
     * @param name Name of the timer to stop
     * @return Duration in seconds
     */
    public static double stopTimer(String name) {
        if (!timers.containsKey(name)) {
            System.out.println("!! Timer not found: " + name + " !!");
            return 0.0;
        }
        
        TimingData timer = timers.get(name);
        double now = Timer.getFPGATimestamp();
        double duration = now - timer.startTime;
        
        // Update stats
        timer.lastDuration = duration;
        timer.totalDuration += duration;
        timer.callCount++;
        
        if (duration < timer.minDuration) {
            timer.minDuration = duration;
        }
        
        if (duration > timer.maxDuration) {
            timer.maxDuration = duration;
        }
        
        // Only update dashboard occasionally to reduce overhead
        updateCounter++;
        if (updateCounter % 10 == 0) {
            updateDashboard();
        }
        
        return duration;
    }
    
    /**
     * Start timing a new robot loop
     */
    public static void startLoopTiming() {
        loopStartTime = Timer.getFPGATimestamp();
    }
    
    /**
     * End loop timing and record metrics
     */
    public static void endLoopTiming() {
        double now = Timer.getFPGATimestamp();
        lastLoopTime = now - loopStartTime;
        loopCounter++;
        
        // Update statistics
        if (lastLoopTime > maxLoopTime) {
            maxLoopTime = lastLoopTime;
        }
        
        // Running average (weighted)
        avgLoopTime = (avgLoopTime * 0.95) + (lastLoopTime * 0.05);
        
        // Check for loop time issues
        if (lastLoopTime > Constants.LOOP_TIME_WARNING) {
            System.out.println("!! SLOW LOOP DETECTED: " + (lastLoopTime * 1000) + 
                              "ms (limit: " + (Constants.LOOP_TIME_WARNING * 1000) + "ms) !!");
        }
    }
    
    /**
     * Update the dashboard with current performance metrics
     */
    private static void updateDashboard() {
        // Update basic metrics
        SmartDashboard.putNumber("Loop_Time_ms", lastLoopTime * 1000);
        SmartDashboard.putNumber("Max_Loop_Time_ms", maxLoopTime * 1000);
        SmartDashboard.putNumber("Avg_Loop_Time_ms", avgLoopTime * 1000);
        SmartDashboard.putNumber("Loop_Frequency", 1.0 / avgLoopTime);
        SmartDashboard.putNumber("Battery_Drain", batteryStartVoltage - RobotController.getBatteryVoltage());
        
        // Only show detailed metrics in verbose mode
        if (verboseMode) {
            for (TimingData timer : timers.values()) {
                String prefix = "Perf_" + timer.name + "_";
                double avgTime = (timer.callCount > 0) ? timer.totalDuration / timer.callCount : 0;
                
                SmartDashboard.putNumber(prefix + "Last_ms", timer.lastDuration * 1000);
                SmartDashboard.putNumber(prefix + "Avg_ms", avgTime * 1000);
                SmartDashboard.putNumber(prefix + "Max_ms", timer.maxDuration * 1000);
                SmartDashboard.putNumber(prefix + "Min_ms", timer.minDuration * 1000);
                SmartDashboard.putNumber(prefix + "Calls", timer.callCount);
            }
        }
    }
    
    /**
     * Get a performance report string
     * 
     * @return Formatted performance report
     */
    public static String getPerformanceReport() {
        StringBuilder report = new StringBuilder();
        report.append("=== PERFORMANCE REPORT ===\n");
        report.append(String.format("Loop Time: %.2fms (avg), %.2fms (max)\n", 
                                   avgLoopTime * 1000, maxLoopTime * 1000));
        report.append(String.format("Loop Frequency: %.1f Hz\n", 1.0 / avgLoopTime));
        report.append(String.format("Battery Drain: %.2fV\n", 
                                   batteryStartVoltage - RobotController.getBatteryVoltage()));
        
        report.append("\nTIMED SECTIONS:\n");
        for (TimingData timer : timers.values()) {
            double avgTime = (timer.callCount > 0) ? timer.totalDuration / timer.callCount : 0;
            report.append(String.format("- %s: %.2fms avg, %.2fms max, %d calls\n", 
                                       timer.name, avgTime * 1000, timer.maxDuration * 1000, timer.callCount));
        }
        
        return report.toString();
    }
    
    /**
     * Reset all performance statistics
     */
    public static void reset() {
        timers.clear();
        loopCounter = 0;
        lastLoopTime = 0.0;
        maxLoopTime = 0.0;
        avgLoopTime = 0.0;
        updateCounter = 0;
        batteryStartVoltage = RobotController.getBatteryVoltage();
        
        System.out.println(">> PERFORMANCE METRICS RESET <<");
    }
}
