// src/main/java/frc/robot/commands/autonomous/basic_path_planning/OptimizedTankDriveCommand.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║ OPTIMIZED TANK DRIVE COMMAND - PRECISION MOVEMENT CONTROL               ║
 * ║══════════════════════════════════════════════════════════════════════════║
 * ║ Advanced movement command implementing time-based precision driving      ║
 * ║ with smooth acceleration profiles and predictive stopping logic.         ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * This command provides high-precision movement for tank drive systems using
 * time-based estimation for position tracking. It's specifically designed to
 * work with PWM motor controllers that lack encoder feedback.
 * 
 * Key Features:
 * - Dynamic acceleration/deceleration profiles for smooth movement
 * - Time-based distance estimation with calibrated speed values
 * - Predictive stopping to minimize overshooting target positions
 * - Status logging and performance monitoring throughout execution
 */
public class OptimizedTankDriveCommand extends Command {
    
    // ===== PHYSICAL PARAMETERS =====
    private static final double WHEEL_CIRCUMFERENCE = Constants.Dimensions.WHEEL_DIAMETER * Math.PI;
    
    // ===== MOTION PROFILE PARAMETERS =====
    private static final double ACCELERATION_TIME = 0.5;  // Time to reach full speed (seconds)
    private static final double DECELERATION_THRESHOLD = 0.4;  // Portion of distance where deceleration begins
    private static final double MIN_POWER = 0.1;  // Minimum power to overcome static friction
    
    // ===== CALIBRATION CONSTANTS =====
    // Time-based motion calibration (replace with encoder feedback when available)
    private static final double METERS_PER_SECOND_AT_FULL_POWER = 3.0;  // Estimated robot speed
    
    // ===== COMMAND STATE VARIABLES =====
    private final DriveSubsystem m_drive;
    private final double m_distance;  // Target distance in meters
    private final double m_maxPower;  // Maximum power (0-1.0)
    private final double m_timeout;   // Maximum execution time (seconds)
    
    // ===== EXECUTION STATE =====
    private Timer m_timer = new Timer();
    private double m_estimatedDistanceTraveled = 0.0;
    private boolean m_isFinished = false;
    
    /**
     * Creates a new precision drive command for straight-line movement.
     * 
     * @param distance Distance to travel in meters (positive = forward, negative = backward)
     * @param maxPower Maximum power level (0 to 1.0)
     * @param timeoutSeconds Maximum execution time in seconds
     */
    public OptimizedTankDriveCommand(double distance, double maxPower, double timeoutSeconds) {
        m_drive = Robot.m_driveSubsystem;
        m_distance = distance;
        m_maxPower = Math.abs(maxPower);
        m_timeout = timeoutSeconds;
        
        // Register the drivetrain requirement
        addRequirements(m_drive);
        
        System.out.println(">> Creating OptimizedTankDriveCommand: " + 
                          distance + "m, " + maxPower + " power, " + 
                          timeoutSeconds + "s timeout");
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public OptimizedTankDriveCommand(double distance, double maxPower) {
        this(distance, maxPower, 10.0);  // Default 10 second timeout
    }
    
    @Override
    public void initialize() {
        System.out.println(">> Starting optimized drive: " + m_distance + " meters");
        
        // Ensure the robot is stopped before we begin
        m_drive.arcadeDrive(0, 0);
        
        // Reset timer and tracking variables
        m_timer.reset();
        m_timer.start();
        m_estimatedDistanceTraveled = 0.0;
        m_isFinished = false;
        
        // NOTE: With encoder support, we would store starting positions here:
        // m_startingLeftPosition = m_drive.getLeftPosition();
        // m_startingRightPosition = m_drive.getRightPosition();
    }
    
    @Override
    public void execute() {
        // Calculate elapsed time for motion profiling
        double elapsedSeconds = m_timer.get();
        
        // Calculate estimated distance traveled based on time and power
        // NOTE: This is a simplified model. With encoders, we would use:
        // double currentLeftPosition = m_drive.getLeftPosition() - m_startingLeftPosition;
        // double currentRightPosition = m_drive.getRightPosition() - m_startingRightPosition;
        // double averageDistance = (currentLeftPosition + currentRightPosition) / 2.0;
        
        // Time-based distance estimation
        double direction = Math.signum(m_distance);
        double currentSpeed = calculateProfiledPower(elapsedSeconds) * m_maxPower * METERS_PER_SECOND_AT_FULL_POWER;
        double distanceDelta = currentSpeed * 0.02; // Assuming 50Hz update rate
        m_estimatedDistanceTraveled += distanceDelta;
        
        // Calculate progress percentage
        double percentComplete = Math.abs(m_estimatedDistanceTraveled / m_distance);
        
        // Calculate power based on motion profile
        double power = calculateProfiledPower(elapsedSeconds) * direction;
        
        // Apply power to drive
        m_drive.tankDrive(power, power);
        
        // Log progress occasionally
        if (elapsedSeconds > 0.5 && Math.floor(elapsedSeconds * 2) / 2.0 == elapsedSeconds) {
            System.out.printf(">> Drive progress: %.1f%% complete (est. %.2fm of %.2fm)\n", 
                             percentComplete * 100.0, 
                             m_estimatedDistanceTraveled,
                             m_distance);
        }
        
        // Check if we've reached the estimated target distance
        if (Math.abs(m_estimatedDistanceTraveled) >= Math.abs(m_distance)) {
            System.out.println(">> Estimated target reached! Stopping");
            m_isFinished = true;
        }
    }
    
    /**
     * Calculate motor power using a trapezoidal motion profile
     * 
     * @param elapsedSeconds Time elapsed since start
     * @return Optimized power level (0.0 to 1.0)
     */
    private double calculateProfiledPower(double elapsedSeconds) {
        double power;
        
        // Acceleration phase - ramp up power
        if (elapsedSeconds < ACCELERATION_TIME) {
            // Linear ramp from MIN_POWER to maxPower
            power = MIN_POWER + (m_maxPower - MIN_POWER) * (elapsedSeconds / ACCELERATION_TIME);
        } 
        // Deceleration phase - slow down as we approach target
        else if (m_estimatedDistanceTraveled / m_distance > (1.0 - DECELERATION_THRESHOLD)) {
            // Calculate how far into deceleration zone we are (0 to 1)
            double decelerationProgress = 
                (m_estimatedDistanceTraveled / m_distance - (1.0 - DECELERATION_THRESHOLD)) / DECELERATION_THRESHOLD;
            
            // Quadratic deceleration curve for smooth stopping
            power = m_maxPower * (1.0 - Math.pow(decelerationProgress, 2)) + MIN_POWER;
        } 
        // Cruise phase - maintain max power
        else {
            power = m_maxPower;
        }
        
        return power;
    }
    
    @Override
    public boolean isFinished() {
        // Check if we've reached target or timed out
        boolean timedOut = m_timer.hasElapsed(m_timeout);
        
        if (timedOut && !m_isFinished) {
            System.out.println(">> Drive command timed out after " + 
                               m_timer.get() + " seconds");
        }
        
        return m_isFinished || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop the motors
        m_drive.arcadeDrive(0, 0);
        
        // Report completion status
        if (interrupted) {
            System.out.println(">> Drive command interrupted");
        } else if (m_isFinished) {
            System.out.println(">> Drive command completed successfully");
            System.out.println(">> Estimated distance traveled: " + 
                               m_estimatedDistanceTraveled + " meters");
        } else {
            System.out.println(">> Drive command timed out");
        }
    }
    
    /**
     * Simulated encoder distance - for compatibility with test harnesses
     * NOTE: This is just a simulation - with real encoders we would return actual positions
     * 
     * @return Estimated distance in meters
     */
    public double getEstimatedDistance() {
        return m_estimatedDistanceTraveled;
    }
}
