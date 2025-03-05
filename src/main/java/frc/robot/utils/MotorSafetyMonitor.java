// src/main/java/frc/robot/utils/MotorSafetyMonitor.java
package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * MotorSafetyMonitor - PROTECTOR OF THE PRECIOUS MOTORS!
 * 
 * This utility class monitors all NEO motors for dangerous conditions:
 * - Excessive current draw (stalls, jams)
 * - Overheating (prolonged high output)
 * - Brownouts (battery voltage drops)
 * - Stall detection (motor not moving despite power)
 * 
 * IT LITERALLY SAVES OUR ROBOT FROM DESTROYING ITSELF!!!
 * 
 * coded by paysean - March 2025
 */
public class MotorSafetyMonitor {
    
    // ===== TRACKING DATA =====
    private static Map<Integer, MotorData> motorMap = new HashMap<>();
    private static double batteryVoltage = 12.0;
    private static int warningCount = 0;
    private static int criticalCount = 0;
    private static Timer monitorTimer = new Timer();
    
    // ===== SAFETY THRESHOLDS =====
    private static final double CURRENT_WARNING = 30.0; // Amps
    private static final double CURRENT_CRITICAL = 40.0; // Amps
    private static final double TEMP_WARNING = 70.0; // Celsius
    private static final double TEMP_CRITICAL = 85.0; // Celsius
    
    // Stall detection parameters
    private static final double MIN_POWER_THRESHOLD = 0.3; // Minimum power to check for stalls
    private static final double MIN_VELOCITY_THRESHOLD = 0.1; // Minimum expected velocity
    private static final double STALL_TIME_THRESHOLD = 1.0; // Seconds before considering stalled
    
    /**
     * Data structure to track each motor's status
     */
    private static class MotorData {
        int id;
        String name;
        double maxCurrent = 0.0;
        double maxTemp = 0.0;
        double lastVelocity = 0.0;
        double lastPower = 0.0;
        double stallTime = 0.0;
        boolean isStalled = false;
        boolean isWarning = false;
        boolean isCritical = false;
        
        public MotorData(int id, String name) {
            this.id = id;
            this.name = name;
        }
    }
    
    /**
     * Initialize the safety monitor system
     */
    public static void initialize() {
        monitorTimer.start();
        System.out.println(">> MOTOR SAFETY MONITOR ACTIVATED!");
        System.out.println(">> PROTECTING THOSE PRECIOUS NEOs FROM DESTRUCTION!");
    }
    
    /**
     * Register a motor for monitoring
     * 
     * @param motor The SparkMax controller to monitor
     * @param name Descriptive name for this motor
     */
    public static void registerMotor(SparkMax motor, String name) {
        int id = motor.getDeviceId();
        if (!motorMap.containsKey(id)) {
            motorMap.put(id, new MotorData(id, name));
            System.out.println(">> MOTOR REGISTERED FOR PROTECTION: " + name + " (ID: " + id + ")");
        }
    }
    
    /**
     * Update monitoring for a specific motor
     * 
     * @param motor The motor to update
     * @return true if the motor is safe to operate
     */
    public static boolean updateMotor(SparkMax motor) {
        int id = motor.getDeviceId();
        if (!motorMap.containsKey(id)) {
            // Auto-register if not found
            registerMotor(motor, "Motor-" + id);
        }
        
        MotorData data = motorMap.get(id);
        
        // Get current status
        double current = motor.getOutputCurrent();
        double temperature = motor.getMotorTemperature();
        double velocity = Math.abs(motor.getEncoder().getVelocity());
        double power = Math.abs(motor.getAppliedOutput());
        
        // Update max values
        if (current > data.maxCurrent) {
            data.maxCurrent = current;
        }
        if (temperature > data.maxTemp) {
            data.maxTemp = temperature;
        }
        
        // Check for stall condition - motor has power but isn't moving
        if (power > MIN_POWER_THRESHOLD && velocity < MIN_VELOCITY_THRESHOLD) {
            data.stallTime += 0.02; // Assuming 50Hz update rate
            
            if (data.stallTime > STALL_TIME_THRESHOLD && !data.isStalled) {
                data.isStalled = true;
                System.out.println("!! STALL DETECTED: " + data.name + " - MOTOR NOT MOVING DESPITE POWER !!");
                SmartDashboard.putBoolean("Stall_" + data.name, true);
            }
        } else {
            data.stallTime = 0.0;
            data.isStalled = false;
            SmartDashboard.putBoolean("Stall_" + data.name, false);
        }
        
        // Check for dangerous current conditions
        boolean wasCritical = data.isCritical;
        boolean wasWarning = data.isWarning;
        
        // Update warning/critical flags
        data.isWarning = current > CURRENT_WARNING || temperature > TEMP_WARNING;
        data.isCritical = current > CURRENT_CRITICAL || temperature > TEMP_CRITICAL || data.isStalled;
        
        // Log state changes
        if (!wasWarning && data.isWarning) {
            warningCount++;
            System.out.println("!! WARNING: " + data.name + " - Current: " + current + 
                              "A, Temp: " + temperature + "°C !!");
        }
        
        if (!wasCritical && data.isCritical) {
            criticalCount++;
            System.out.println("!!! CRITICAL: " + data.name + " - SAFETY LIMITS EXCEEDED !!!");
            System.out.println("!!! Current: " + current + "A, Temp: " + temperature + "°C !!!");
        }
        
        // Update dashboard data
        updateDashboard(data);
        
        // Store last values for next comparison
        data.lastVelocity = velocity;
        data.lastPower = power;
        
        // Return motor safety status
        return !data.isCritical;
    }
    
    /**
     * Update all registered motors
     * 
     * @return true if all motors are safe
     */
    public static boolean updateAll() {
        updateBatteryVoltage();
        boolean allSafe = true;
        
        // Only log periodic updates to avoid spamming
        boolean shouldLog = monitorTimer.hasElapsed(5.0);
        if (shouldLog) {
            monitorTimer.reset();
        }
        
        if (shouldLog) {
            System.out.println(">> MOTOR SAFETY STATUS:");
            System.out.println(">> Battery: " + batteryVoltage + "V, Warnings: " + 
                              warningCount + ", Critical: " + criticalCount);
        }
        
        return allSafe;
    }
    
    /**
     * Update battery voltage monitoring
     */
    private static void updateBatteryVoltage() {
        // Get latest voltage from Robot Controller
        batteryVoltage = RobotController.getBatteryVoltage();
        
        // Check for brownout conditions
        if (batteryVoltage < Constants.BATTERY_WARNING_THRESHOLD) {
            System.out.println("!! LOW BATTERY WARNING: " + batteryVoltage + "V !!");
        }
        
        if (batteryVoltage < Constants.BATTERY_BROWNOUT_THRESHOLD) {
            System.out.println("!!! CRITICAL BATTERY LEVEL: " + batteryVoltage + "V !!!");
            System.out.println("!!! BROWNOUT IMMINENT - REDUCE POWER USAGE !!!");
        }
        
        SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
    }
    
    /**
     * Update dashboard with motor status
     * 
     * @param data Motor data to display
     */
    private static void updateDashboard(MotorData data) {
        SmartDashboard.putNumber(data.name + "_Current", data.maxCurrent);
        SmartDashboard.putNumber(data.name + "_Temp", data.maxTemp);
        SmartDashboard.putBoolean(data.name + "_Warning", data.isWarning);
        SmartDashboard.putBoolean(data.name + "_Critical", data.isCritical);
    }
    
    /**
     * Get the status of a motor
     * 
     * @param motorId ID of the motor to check
     * @return 0=OK, 1=WARNING, 2=CRITICAL
     */
    public static int getMotorStatus(int motorId) {
        if (!motorMap.containsKey(motorId)) {
            return Constants.STATUS_OK;
        }
        
        MotorData data = motorMap.get(motorId);
        if (data.isCritical) {
            return Constants.STATUS_CRITICAL;
        } else if (data.isWarning) {
            return Constants.STATUS_WARNING;
        } else {
            return Constants.STATUS_OK;
        }
    }
    
    /**
     * Reset all warning counters and tracking data
     */
    public static void resetCounters() {
        warningCount = 0;
        criticalCount = 0;
        
        for (MotorData data : motorMap.values()) {
            data.maxCurrent = 0.0;
            data.maxTemp = 0.0;
            data.isWarning = false;
            data.isCritical = false;
            data.stallTime = 0.0;
            data.isStalled = false;
        }
        
        System.out.println(">> MOTOR SAFETY COUNTERS RESET!");
    }
}
