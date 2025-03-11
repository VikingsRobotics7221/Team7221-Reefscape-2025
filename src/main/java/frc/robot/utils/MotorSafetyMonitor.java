// src/main/java/frc/robot/utils/MotorSafetyMonitor.java
package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════╗
 * ║ MOTOR SAFETY MONITOR - COMPREHENSIVE PROTECTION SYSTEM            ║
 * ║═══════════════════════════════════════════════════════════════════║
 * ║ Advanced monitoring system that protects motors from:             ║
 * ║  • Thermal damage                                                 ║
 * ║  • Current overloads                                              ║
 * ║  • Stall conditions                                               ║
 * ║  • Brown-out scenarios                                            ║
 * ╚═══════════════════════════════════════════════════════════════════╝
 * 
 * This utility provides a robust monitoring framework that detects and prevents
 * conditions that could damage motors or impair robot performance. The system
 * operates independently from specific motor controller implementations to
 * ensure flexibility and reliability across various hardware configurations.
 * 
 * IMPLEMENTATION ARCHITECTURE:
 * - Hardware-agnostic design for multi-platform support
 * - Event-based alert system with tiered severity levels
 * - Real-time dashboard integration for instant operator feedback
 * - Proactive detection of fault conditions before damage occurs
 * 
 * FAULT DETECTION CAPABILITIES:
 * - Current detection: Identifies when motors draw excessive current
 * - Temperature monitoring: Prevents thermal damage to motor windings
 * - Stall detection: Recognizes when motors are powered but not moving
 * - Voltage monitoring: Identifies brownout conditions that affect performance
 */
public class MotorSafetyMonitor {
    
    // ========== SYSTEM CONSTANTS ==========
    // Alert thresholds for various monitored parameters
    private static final double CURRENT_WARNING = 30.0;  // Amps
    private static final double CURRENT_CRITICAL = 40.0; // Amps
    private static final double TEMP_WARNING = 70.0;     // Celsius
    private static final double TEMP_CRITICAL = 85.0;    // Celsius
    private static final double STALL_TIME_THRESHOLD = 1.0; // Seconds
    
    // ========== SYSTEM STATE TRACKING ==========
    private static Map<String, MotorData> motorRegistry = new HashMap<>();
    private static double batteryVoltage = 12.0;
    private static int warningCount = 0;
    private static int criticalCount = 0;
    private static Timer monitorTimer = new Timer();
    private static boolean systemInitialized = false;

    /**
     * Comprehensive data structure for tracking motor health metrics
     */
    private static class MotorData {
        // Motor identification
        String name;          // Descriptive identifier for this motor
        
        // Operational metrics
        double maxCurrent;    // Highest current observed
        double maxTemp;       // Highest temperature observed 
        double stallTime;     // Duration of potential stall condition
        
        // Status flags
        boolean isStalled;    // Whether motor is currently stalled
        boolean isWarning;    // Whether motor is in warning state
        boolean isCritical;   // Whether motor is in critical state
        
        // Tracking metrics
        int faultCount;       // Number of fault conditions detected
        long lastUpdateTime;  // Timestamp of last update
        
        /**
         * Creates a comprehensive motor tracking object
         * 
         * @param name Descriptive name for logging and display
         */
        public MotorData(String name) {
            this.name = name;
            this.maxCurrent = 0.0;
            this.maxTemp = 0.0;
            this.stallTime = 0.0;
            this.isStalled = false;
            this.isWarning = false;
            this.isCritical = false;
            this.faultCount = 0;
            this.lastUpdateTime = System.currentTimeMillis();
        }
    }
    
    /**
     * Initialize the motor safety monitoring system.
     * This must be called once during robot initialization.
     */
    public static void initialize() {
        // System preparation
        motorRegistry.clear();
        monitorTimer.reset();
        monitorTimer.start();
        systemInitialized = true;
        warningCount = 0;
        criticalCount = 0;
        
        // System initialization notification
        System.out.println("╔═════════════════════════════════════════════════╗");
        System.out.println("║  MOTOR SAFETY MONITOR INITIALIZED               ║");
        System.out.println("║  Protecting your robot from electrical damage   ║");
        System.out.println("╚═════════════════════════════════════════════════╝");
    }
    
    /**
     * Register a motor for monitoring by name.
     * 
     * @param motorName Unique identifier for this motor
     */
    public static void registerMotor(String motorName) {
        if (!systemInitialized) {
            System.err.println("ERROR: MotorSafetyMonitor not initialized! Call initialize() first.");
            return;
        }
        
        if (!motorRegistry.containsKey(motorName)) {
            motorRegistry.put(motorName, new MotorData(motorName));
            System.out.println(">> Motor registered for monitoring: " + motorName);
        }
    }
    
    /**
     * Update the safety status of a specific motor with current metrics.
     * 
     * @param motorName The name of the motor to update
     * @param current Current draw in amps
     * @param temperature Motor temperature in Celsius
     * @param isMoving Whether the motor is currently in motion
     * @param appliedPower Power level being applied (-1.0 to 1.0)
     * @return true if the motor is safe to operate, false if critical condition detected
     */
    public static boolean updateMotorStatus(String motorName, double current, double temperature, 
                                           boolean isMoving, double appliedPower) {
        // Validate system state
        if (!systemInitialized) {
            System.err.println("ERROR: MotorSafetyMonitor not initialized! Call initialize() first.");
            return false;
        }
        
        // Auto-register if this is a new motor
        if (!motorRegistry.containsKey(motorName)) {
            registerMotor(motorName);
        }
        
        MotorData data = motorRegistry.get(motorName);
        data.lastUpdateTime = System.currentTimeMillis();
        
        // Update peak values for tracking
        if (current > data.maxCurrent) {
            data.maxCurrent = current;
        }
        if (temperature > data.maxTemp) {
            data.maxTemp = temperature;
        }
        
        // Check for stall condition - motor has power but isn't moving
        if (Math.abs(appliedPower) > 0.2 && !isMoving) {
            data.stallTime += 0.02; // Assuming 50Hz update rate
            
            if (data.stallTime > STALL_TIME_THRESHOLD && !data.isStalled) {
                data.isStalled = true;
                data.faultCount++;
                System.out.println("!! STALL DETECTED: " + data.name + " - MOTOR NOT MOVING !!");
                SmartDashboard.putBoolean("Stall_" + data.name, true);
            }
        } else {
            data.stallTime = 0.0;
            data.isStalled = false;
            SmartDashboard.putBoolean("Stall_" + data.name, false);
        }
        
        // Check for warning/critical conditions
        boolean wasWarning = data.isWarning;
        boolean wasCritical = data.isCritical;
        
        // Update status flags based on current measurements
        data.isWarning = current > CURRENT_WARNING || temperature > TEMP_WARNING;
        data.isCritical = current > CURRENT_CRITICAL || temperature > TEMP_CRITICAL || data.isStalled;
        
        // Log state transitions for operator awareness
        if (!wasWarning && data.isWarning) {
            warningCount++;
            System.out.println("!! WARNING: " + data.name + " - Current: " + 
                              String.format("%.1f", current) + "A, Temp: " + 
                              String.format("%.1f", temperature) + "°C");
        }
        
        if (!wasCritical && data.isCritical) {
            criticalCount++;
            data.faultCount++;
            System.out.println("!!! CRITICAL: " + data.name + " - SAFETY LIMITS EXCEEDED !!!");
            System.out.println("!!! Current: " + String.format("%.1f", current) + 
                             "A, Temp: " + String.format("%.1f", temperature) + "°C");
        }
        
        // Update dashboard with current status
        updateDashboard(data);
        
        // Return operational safety status
        return !data.isCritical;
    }
    
    /**
     * Perform a complete system update, checking all monitored parameters
     * including battery voltage. This should be called periodically in robotPeriodic().
     * 
     * @return true if all systems are within safe operating parameters
     */
    public static boolean update() {
        // Validate system state
        if (!systemInitialized) {
            System.err.println("ERROR: MotorSafetyMonitor not initialized! Call initialize() first.");
            return false;
        }
        
        // Monitor battery voltage
        updateBatteryVoltage();
        
        // Periodic system status reporting
        if (monitorTimer.hasElapsed(5.0)) {
            monitorTimer.reset();
            
            // Only log if we have registered motors
            if (!motorRegistry.isEmpty()) {
                System.out.println(">> MOTOR SAFETY STATUS:");
                System.out.println(">> Battery: " + String.format("%.2f", batteryVoltage) + "V, Warnings: " + 
                                warningCount + ", Critical: " + criticalCount);
                
                // Report on stale data (motors not recently updated)
                checkForStaleData();
            }
        }
        
        // Check for any critical conditions
        boolean allSafe = true;
        for (MotorData data : motorRegistry.values()) {
            if (data.isCritical) {
                allSafe = false;
                break;
            }
        }
        
        return allSafe;
    }
    
    /**
     * Check for motors that haven't been updated recently
     */
    private static void checkForStaleData() {
        long currentTime = System.currentTimeMillis();
        List<String> staleMotors = new ArrayList<>();
        
        for (Map.Entry<String, MotorData> entry : motorRegistry.entrySet()) {
            MotorData data = entry.getValue();
            // Check if data is older than 1 second
            if (currentTime - data.lastUpdateTime > 1000) {
                staleMotors.add(data.name);
            }
        }
        
        if (!staleMotors.isEmpty()) {
            System.out.println(">> WARNING: Stale data for motors: " + String.join(", ", staleMotors));
        }
    }
    
    /**
     * Update battery voltage and check for brownout conditions
     */
    private static void updateBatteryVoltage() {
        // Get latest battery voltage from robot controller
        batteryVoltage = RobotController.getBatteryVoltage();
        
        // Check for warning conditions
        if (batteryVoltage < Constants.Performance.BATTERY_WARNING_THRESHOLD) {
            System.out.println("!! LOW BATTERY WARNING: " + String.format("%.2f", batteryVoltage) + "V !!");
        }
        
        // Check for critical brownout conditions
        if (batteryVoltage < Constants.Performance.BATTERY_BROWNOUT_THRESHOLD) {
            System.out.println("!!! CRITICAL BATTERY LEVEL: " + String.format("%.2f", batteryVoltage) + "V !!!");
            System.out.println("!!! BROWNOUT IMMINENT - REDUCE POWER USAGE !!!");
        }
        
        // Update dashboard
        SmartDashboard.putNumber("Battery_Voltage", batteryVoltage);
    }
    
    /**
     * Update SmartDashboard with current motor status
     * 
     * @param data Motor data to display
     */
    private static void updateDashboard(MotorData data) {
        SmartDashboard.putNumber(data.name + "_Current", data.maxCurrent);
        SmartDashboard.putNumber(data.name + "_Temp", data.maxTemp);
        SmartDashboard.putBoolean(data.name + "_Warning", data.isWarning);
        SmartDashboard.putBoolean(data.name + "_Critical", data.isCritical);
        SmartDashboard.putNumber(data.name + "_FaultCount", data.faultCount);
    }
    
    /**
     * Get the safety status of a motor
     * 
     * @param motorName Name of the motor to check
     * @return Status code: 0=OK, 1=WARNING, 2=CRITICAL
     */
    public static int getMotorStatus(String motorName) {
        // Validate system state
        if (!systemInitialized) {
            return Constants.Performance.STATUS_WARNING; // Return warning if system not initialized
        }
        
        // Check if motor is registered
        if (!motorRegistry.containsKey(motorName)) {
            return Constants.Performance.STATUS_OK; // Unknown motors assumed OK
        }
        
        // Return appropriate status based on conditions
        MotorData data = motorRegistry.get(motorName);
        if (data.isCritical) {
            return Constants.Performance.STATUS_CRITICAL;
        } else if (data.isWarning) {
            return Constants.Performance.STATUS_WARNING;
        } else {
            return Constants.Performance.STATUS_OK;
        }
    }
    
    /**
     * Reset all monitoring counters and tracking data.
     * Call this when transitioning between robot modes.
     */
    public static void resetCounters() {
        // Validate system state
        if (!systemInitialized) {
            System.err.println("ERROR: MotorSafetyMonitor not initialized! Call initialize() first.");
            return;
        }
        
        // Reset global counters
        warningCount = 0;
        criticalCount = 0;
        
        // Reset data for each motor
        for (MotorData data : motorRegistry.values()) {
            data.maxCurrent = 0.0;
            data.maxTemp = 0.0;
            data.isWarning = false;
            data.isCritical = false;
            data.stallTime = 0.0;
            data.isStalled = false;
        }
        
        System.out.println(">> MOTOR SAFETY MONITORING RESET - ALL COUNTERS CLEARED");
    }
    
    /**
     * Simplified method for checking motor safety system.
     * Call this from subsystem periodic methods for continuous monitoring.
     */
    public static void check() {
        update();
    }
}
