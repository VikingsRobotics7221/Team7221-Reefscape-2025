// src/main/java/frc/robot/utils/PerformanceDashboard.java
package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * ╔════════════════════════════════════════════════════════════════════════════╗
 * ║ PERFORMANCE DASHBOARD - REAL-TIME EXECUTION OPTIMIZATION SYSTEM            ║
 * ║════════════════════════════════════════════════════════════════════════════║
 * ║ Measures, analyzes, and visualizes code execution performance metrics      ║
 * ║ to identify bottlenecks and ensure optimal robot control responsiveness.   ║
 * ╚════════════════════════════════════════════════════════════════════════════╝
 * 
 * This comprehensive performance monitoring system provides critical visibility into:
 * 
 *   1. EXECUTION TIMES - Tracks how long each code section takes to run
 *   2. LOOP FREQUENCY - Monitors robot control loop execution rate
 *   3. CPU UTILIZATION - Identifies processing bottlenecks
 *   4. BATTERY EFFICIENCY - Correlates power consumption with code execution
 * 
 * HOW TO USE:
 *   1. Call initialize() during robotInit()
 *   2. Call startTimer("name") at the beginning of a section
 *   3. Call stopTimer("name") at the end to record the duration
 *   4. Call startLoopTiming() at the beginning of robotPeriodic()
 *   5. Call endLoopTiming() at the end of robotPeriodic()
 * 
 * DASHBOARD INTEGRATION:
 *   - Real-time metrics available on SmartDashboard
 *   - Performance report accessible through getPerformanceReport()
 */
public class PerformanceDashboard {
    
    // ===== PERFORMANCE THRESHOLDS =====
    // Define locally to avoid dependency issues
    private static final double LOOP_TIME_WARNING_THRESHOLD = 0.018; // 18ms loop time warning threshold
    
    // ===== TIMER DATA STORAGE =====
    private static Map<String, TimingData> timers = new HashMap<>();
    private static double loopStartTime = 0.0;
    private static int loopCounter = 0;
    private static double lastLoopTime = 0.0;
    private static double maxLoopTime = 0.0;
    private static double avgLoopTime = 0.0;
    private static double batteryStartVoltage = 12.0;
    
    // ===== DISPLAY CONFIGURATION =====
    private static boolean verboseMode = false;
    private static AtomicInteger updateCounter = new AtomicInteger(0);
    private static long startTimeMillis = 0;
    
    // ===== PERFORMANCE TRACKING =====
    private static int slowLoopCount = 0;
    private static List<String> slowestSections = new ArrayList<>();
    private static double[] loopTimeHistory = new double[50]; // Rolling history
    private static int historyIndex = 0;
    private static boolean systemInitialized = false;
    
    /**
     * Data structure for tracking timing metrics for each monitored section of code
     */
    private static class TimingData {
        String name;            // Section identifier
        double startTime;       // When timing was started
        double lastDuration;    // Most recent execution time
        double totalDuration;   // Cumulative execution time
        double minDuration;     // Fastest execution time
        double maxDuration;     // Slowest execution time
        int callCount;          // Number of times section was executed
        long lastUpdateMs;      // System time of last update
        
        /**
         * Creates a new timing data object for a code section
         * 
         * @param name Identifier for this code section
         */
        public TimingData(String name) {
            this.name = name;
            this.startTime = 0;
            this.lastDuration = 0;
            this.totalDuration = 0;
            this.minDuration = Double.MAX_VALUE;
            this.maxDuration = 0;
            this.callCount = 0;
            this.lastUpdateMs = System.currentTimeMillis();
        }
        
        /**
         * Calculate average duration of this timed section
         * 
         * @return Average time in seconds
         */
        public double getAverageDuration() {
            return callCount > 0 ? totalDuration / callCount : 0.0;
        }
        
        /**
         * Reset all timing statistics for this section
         */
        public void reset() {
            totalDuration = 0.0;
            minDuration = Double.MAX_VALUE;
            maxDuration = 0.0;
            callCount = 0;
            lastDuration = 0.0;
            lastUpdateMs = System.currentTimeMillis();
        }
    }
    
    /**
     * Initialize the performance dashboard
     * 
     * @param verbose Whether to show detailed metrics (more dashboard entries)
     */
    public static void initialize(boolean verbose) {
        // Set system configuration
        verboseMode = verbose;
        loopStartTime = Timer.getFPGATimestamp();
        batteryStartVoltage = RobotController.getBatteryVoltage();
        startTimeMillis = System.currentTimeMillis();
        
        // Initialize tracking arrays
        for (int i = 0; i < loopTimeHistory.length; i++) {
            loopTimeHistory[i] = 0.0;
        }
        
        // Reset counters
        loopCounter = 0;
        slowLoopCount = 0;
        updateCounter.set(0);
        historyIndex = 0;
        
        // Indicate system is ready for use
        systemInitialized = true;
        
        // Dashboard indicator
        SmartDashboard.putBoolean("Performance_Tracking", true);
        
        // Log initialization
        System.out.println("\n" +
            "╔════════════════════════════════════════════════════╗\n" +
            "║   PERFORMANCE DASHBOARD ACTIVATED                  ║\n" +
            "║   OPTIMIZATION MODE: " + (verbose ? "MAXIMUM DETAIL" : "STANDARD") + "           ║\n" +
            "╚════════════════════════════════════════════════════╝");
    }
    
    /**
     * Reset all performance metrics
     * Call this when switching modes (auto → teleop)
     */
    public static void reset() {
        // Skip if not initialized
        if (!systemInitialized) {
            System.err.println("ERROR: PerformanceDashboard not initialized. Call initialize() first!");
            return;
        }
        
        // Reset loop metrics
        maxLoopTime = 0.0;
        avgLoopTime = 0.0;
        loopCounter = 0;
        slowLoopCount = 0;
        batteryStartVoltage = RobotController.getBatteryVoltage();
        
        // Reset timing data for all sections
        for (TimingData timer : timers.values()) {
            timer.reset();
        }
        
        // Clear history tracking
        for (int i = 0; i < loopTimeHistory.length; i++) {
            loopTimeHistory[i] = 0.0;
        }
        historyIndex = 0;
        
        // Reset slowest sections list
        slowestSections.clear();
        
        System.out.println(">> PERFORMANCE METRICS RESET <<");
    }
    
    /**
     * Start a timer for a specific code section
     * 
     * @param name Name of the code section to time
     */
    public static void startTimer(String name) {
        // Skip if not initialized
        if (!systemInitialized) {
            System.err.println("ERROR: PerformanceDashboard not initialized. Call initialize() first!");
            return;
        }
        
        // Create new timer if needed
        if (!timers.containsKey(name)) {
            timers.put(name, new TimingData(name));
        }
        
        // Start timing this section
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
        // Skip if not initialized
        if (!systemInitialized) {
            System.err.println("ERROR: PerformanceDashboard not initialized. Call initialize() first!");
            return 0.0;
        }
        
        // Validate timer exists
        if (!timers.containsKey(name)) {
            System.out.println("!! Timer not found: " + name + " !!");
            return 0.0;
        }
        
        // Get timer and calculate duration
        TimingData timer = timers.get(name);
        double now = Timer.getFPGATimestamp();
        double duration = now - timer.startTime;
        
        // Update statistics
        timer.lastDuration = duration;
        timer.totalDuration += duration;
        timer.callCount++;
        timer.lastUpdateMs = System.currentTimeMillis();
        
        // Update min/max values
        if (duration < timer.minDuration) {
            timer.minDuration = duration;
        }
        
        if (duration > timer.maxDuration) {
            timer.maxDuration = duration;
        }
        
        // Check if this section is taking too long (potential bottleneck)
        if (duration > LOOP_TIME_WARNING_THRESHOLD) {
            if (!slowestSections.contains(name)) {
                if (slowestSections.size() >= 5) {
                    slowestSections.remove(0); // Keep list manageable
                }
                slowestSections.add(name);
            }
        }
        
        // Only update dashboard occasionally to reduce overhead
        if (updateCounter.incrementAndGet() % 10 == 0) {
            updateDashboard();
        }
        
        return duration;
    }
    
    /**
     * Start timing a new robot loop iteration
     * Call at beginning of robotPeriodic()
     */
    public static void startLoopTiming() {
        // Skip if not initialized
        if (!systemInitialized) {
            return;
        }
        
        loopStartTime = Timer.getFPGATimestamp();
    }
    
    /**
     * End loop timing and record metrics
     * Call at end of robotPeriodic()
     */
    public static void endLoopTiming() {
        // Skip if not initialized
        if (!systemInitialized) {
            return;
        }
        
        // Calculate loop execution time
        double now = Timer.getFPGATimestamp();
        lastLoopTime = now - loopStartTime;
        loopCounter++;
        
        // Store in history array for jitter analysis
        loopTimeHistory[historyIndex] = lastLoopTime;
        historyIndex = (historyIndex + 1) % loopTimeHistory.length;
        
        // Update statistics
        if (lastLoopTime > maxLoopTime) {
            maxLoopTime = lastLoopTime;
        }
        
        // Running average (weighted)
        if (avgLoopTime == 0.0) {
            avgLoopTime = lastLoopTime;
        } else {
            avgLoopTime = (avgLoopTime * 0.95) + (lastLoopTime * 0.05);
        }
        
        // Check for loop time issues
        if (lastLoopTime > LOOP_TIME_WARNING_THRESHOLD) {
            slowLoopCount++;
            if (slowLoopCount % 5 == 0) { // Don't spam warnings
                System.out.println("!! SLOW LOOP DETECTED: " + 
                                 String.format("%.2f", lastLoopTime * 1000) + 
                                 "ms (limit: " + 
                                 String.format("%.2f", LOOP_TIME_WARNING_THRESHOLD * 1000) + "ms) !!");
            }
        }
    }
    
    /**
     * Update the dashboard with current performance metrics
     */
    private static void updateDashboard() {
        // Update core metrics
        SmartDashboard.putNumber("Loop_Time_ms", lastLoopTime * 1000);
        SmartDashboard.putNumber("Max_Loop_Time_ms", maxLoopTime * 1000);
        SmartDashboard.putNumber("Avg_Loop_Time_ms", avgLoopTime * 1000);
        SmartDashboard.putNumber("Loop_Frequency", 1.0 / avgLoopTime);
        SmartDashboard.putNumber("Battery_Drain", batteryStartVoltage - RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Run_Time_Minutes", (System.currentTimeMillis() - startTimeMillis) / 60000.0);
        
        // Only show detailed metrics in verbose mode
        if (verboseMode) {
            for (TimingData timer : timers.values()) {
                String prefix = "Perf_" + timer.name + "_";
                
                SmartDashboard.putNumber(prefix + "Last_ms", timer.lastDuration * 1000);
                SmartDashboard.putNumber(prefix + "Avg_ms", timer.getAverageDuration() * 1000);
                SmartDashboard.putNumber(prefix + "Max_ms", timer.maxDuration * 1000);
                SmartDashboard.putNumber(prefix + "Min_ms", timer.minDuration * 1000);
                SmartDashboard.putNumber(prefix + "Calls", timer.callCount);
            }
            
            // Calculate jitter (variability in loop time)
            double jitter = calculateJitter();
            SmartDashboard.putNumber("Loop_Jitter_ms", jitter * 1000);
        }
    }
    
    /**
     * Calculate loop time jitter (variability)
     * 
     * @return Standard deviation of loop times
     */
    private static double calculateJitter() {
        double sum = 0.0;
        int count = 0;
        
        // Calculate mean of valid entries
        for (double time : loopTimeHistory) {
            if (time > 0.0) {
                sum += time;
                count++;
            }
        }
        
        if (count < 2) return 0.0;
        
        double mean = sum / count;
        
        // Calculate variance
        double variance = 0.0;
        for (double time : loopTimeHistory) {
            if (time > 0.0) {
                variance += Math.pow(time - mean, 2);
            }
        }
        
        variance /= (count - 1);
        
        // Return standard deviation
        return Math.sqrt(variance);
    }
    
    /**
     * Get a formatted performance report string
     * Great for logging overall performance
     * 
     * @return Formatted performance report
     */
    public static String getPerformanceReport() {
        // Skip if not initialized
        if (!systemInitialized) {
            return "ERROR: PerformanceDashboard not initialized. Call initialize() first!";
        }
        
        StringBuilder report = new StringBuilder();
        report.append("╔═══════════════ PERFORMANCE REPORT ════════════════╗\n");
        report.append(String.format("║ Loop Time: %.2fms (avg), %.2fms (max)            ║\n", 
                                  avgLoopTime * 1000, maxLoopTime * 1000));
        report.append(String.format("║ Loop Frequency: %.1f Hz                          ║\n", 
                                  1.0 / avgLoopTime));
        report.append(String.format("║ Slow Loops: %d of %d total iterations            ║\n", 
                                  slowLoopCount, loopCounter));
        report.append(String.format("║ Battery Drain: %.2fV                              ║\n", 
                                  batteryStartVoltage - RobotController.getBatteryVoltage()));
        report.append(String.format("║ Run Time: %.1f minutes                           ║\n",
                                  (System.currentTimeMillis() - startTimeMillis) / 60000.0));
        report.append("╚════════════════════════════════════════════════════╝\n");
        
        // Add slowest code sections
        if (!slowestSections.isEmpty()) {
            report.append("\n>> SLOWEST CODE SECTIONS:\n");
            for (String section : slowestSections) {
                TimingData timer = timers.get(section);
                if (timer != null) {
                    report.append(String.format("   %s: %.2fms avg, %.2fms max\n", 
                                             timer.name, 
                                             timer.getAverageDuration() * 1000, 
                                             timer.maxDuration * 1000));
                }
            }
        }
        
        // Add section timing details
        report.append("\n>> ALL TIMED SECTIONS:\n");
        List<TimingData> sortedTimers = new ArrayList<>(timers.values());
        sortedTimers.sort((a, b) -> Double.compare(b.getAverageDuration(), a.getAverageDuration()));
        
        for (TimingData timer : sortedTimers) {
            if (timer.callCount > 0) {
                report.append(String.format("   %s: %.2fms avg, %.2fms max, %d calls\n", 
                                         timer.name, 
                                         timer.getAverageDuration() * 1000, 
                                         timer.maxDuration * 1000,
                                         timer.callCount));
            }
        }
        
        return report.toString();
    }
    
    /**
     * Get loop timing summary - useful for quick status checks
     * 
     * @return Short timing summary string
     */
    public static String getTimingSummary() {
        // Skip if not initialized
        if (!systemInitialized) {
            return "ERROR: PerformanceDashboard not initialized";
        }
        
        return String.format("Loop: %.2fms avg, %.2fms max, %.1f Hz, %d slow loops", 
                          avgLoopTime * 1000, 
                          maxLoopTime * 1000,
                          1.0 / avgLoopTime,
                          slowLoopCount);
    }
    
    /**
     * Get the current loop frequency
     * 
     * @return Loop frequency in Hz
     */
    public static double getLoopFrequency() {
        return 1.0 / avgLoopTime;
    }
    
    /**
     * Get the maximum recorded loop time
     * 
     * @return Maximum loop time in seconds
     */
    public static double getMaxLoopTime() {
        return maxLoopTime;
    }
    
    /**
     * Get the average loop time
     * 
     * @return Average loop time in seconds
     */
    public static double getAverageLoopTime() {
        return avgLoopTime;
    }
    
    /**
     * Check if any section has excessive execution time
     * 
     * @return true if any section exceeds warning threshold
     */
    public static boolean hasPerformanceIssue() {
        // Skip if not initialized
        if (!systemInitialized) {
            return false;
        }
        
        // Check loop time first
        if (avgLoopTime > LOOP_TIME_WARNING_THRESHOLD) {
            return true;
        }
        
        // Check individual sections
        for (TimingData timer : timers.values()) {
            if (timer.maxDuration > LOOP_TIME_WARNING_THRESHOLD && timer.callCount > 5) {
                return true;
            }
        }
        
        return false;
    }
    
    /**
     * Start a timing profile for a specific subsystem processing cycle.
     * Useful for tracking subsystem periodic() method performance.
     * 
     * @param subsystemName Name of the subsystem
     */
    public static void startSubsystemTiming(String subsystemName) {
        startTimer("Subsystem_" + subsystemName);
    }
    
    /**
     * End timing profile for a subsystem processing cycle
     * 
     * @param subsystemName Name of the subsystem
     * @return Duration in seconds
     */
    public static double endSubsystemTiming(String subsystemName) {
        return stopTimer("Subsystem_" + subsystemName);
    }
}
