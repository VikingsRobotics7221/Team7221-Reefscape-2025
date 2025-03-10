// src/main/java/frc/robot/commands/autonomous/basic_path_planning/OptimizedTankDriveCommand.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * OptimizedTankDriveCommand - Precision Movement Control for Tank Drive
 * 
 * This command provides precise, position-controlled movement for the robot's
 * tank drive system. It's specially calibrated for the 16:1 gear ratio to ensure
 * accurate distance tracking and smooth acceleration/deceleration.
 * 
 * Features:
 * - Dynamic acceleration/deceleration profiles for smooth movement
 * - Encoder-based position tracking for exact distances
 * - Automatic correction for drivetrain drift during straight-line movement
 * - Predictive stopping to minimize overshooting target positions
 * 
 * System Integration:
 * - Uses DriveSubsystem for motor control
 * - Uses Constants for configuration parameters
 * - Uses encoder feedback for closed-loop position control
 */
public class OptimizedTankDriveCommand extends Command {
    
    // Physical parameters
    private static final double WHEEL_CIRCUMFERENCE = Constants.Dimensions.WHEEL_DIAMETER * Math.PI;
    
    // Motion profile parameters
    private static final double ACCELERATION_TIME = 0.5;  // Time to reach full speed (seconds)
    private static final double DECELERATION_THRESHOLD = 0.4;  // Portion of distance where deceleration begins
    private static final double MIN_POWER = 0.1;  // Minimum power to overcome static friction
    private static final double ENCODER_TOLERANCE = 0.05;  // Acceptable position error (rotations)
    
    // Command state variables
    private final DriveSubsystem m_drive;
    private final double m_distance;  // Target distance in meters
    private final double m_maxPower;  // Maximum power (0-1.0)
    private final double m_timeout;   // Maximum execution time (seconds)
    
    // Target tracking
    private double m_targetEncoderDistance;  // Target distance in encoder rotations
    private double m_startingLeftPosition;
    private double m_startingRightPosition;
    
    // Execution state
    private long m_startTimeMillis;
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
        
        // Calculate target encoder distance (rotations)
        m_targetEncoderDistance = distance / WHEEL_CIRCUMFERENCE;
        
        // Register the drivetrain requirement
        addRequirements(m_drive);
        
        System.out.println("Creating OptimizedTankDriveCommand: " + 
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
        System.out.println("Starting optimized drive: " + m_distance + " meters");
        
        // Ensure the robot is stopped before we begin
        m_drive.arcadeDrive(0, 0);
        
        // Record starting positions for distance tracking
        m_startingLeftPosition = m_drive.getLeftPosition();
        m_startingRightPosition = m_drive.getRightPosition();
        
        // Record start time for timeout and motion profiling
        m_startTimeMillis = System.currentTimeMillis();
        
        // Reset completion state
        m_isFinished = false;
    }
    
    @Override
    public void execute() {
        // Calculate elapsed time for motion profiling
        double elapsedSeconds = (System.currentTimeMillis() - m_startTimeMillis) / 1000.0;
        
        // Get current positions and calculate distance traveled
        double currentLeftPosition = m_drive.getLeftPosition() - m_startingLeftPosition;
        double currentRightPosition = m_drive.getRightPosition() - m_startingRightPosition;
        
        // Calculate average distance traveled (preserving direction)
        double direction = Math.signum(m_targetEncoderDistance);
        double averageDistance = direction * (Math.abs(currentLeftPosition) + 
                                            Math.abs(currentRightPosition)) / 2.0;
        
        // Calculate progress percentage
        double percentComplete = Math.abs(averageDistance / m_targetEncoderDistance);
        
        // Calculate power based on motion profile
        double power = calculateProfiledPower(elapsedSeconds, percentComplete);
        
        // Apply direction
        power *= direction;
        
        // Apply small correction if sides are uneven (drift compensation)
        double leftAdjust = 1.0;
        double rightAdjust = 1.0;
        
        // If one side is moving faster than the other, adjust to correct drift
        double leftDistance = Math.abs(currentLeftPosition);
        double rightDistance = Math.abs(currentRightPosition);
        
        if (leftDistance > rightDistance * 1.02) {
            // Left side is moving faster, reduce its power slightly
            leftAdjust = 0.95;
            rightAdjust = 1.05;
        } else if (rightDistance > leftDistance * 1.02) {
            // Right side is moving faster, reduce its power slightly
            leftAdjust = 1.05;
            rightAdjust = 0.95;
        }
        
        // Apply adjusted power to both sides
        m_drive.tankDrive(power * leftAdjust, power * rightAdjust);
        
        // Log progress occasionally
        if (elapsedSeconds > 0.5 && Math.floor(elapsedSeconds * 2) / 2.0 == elapsedSeconds) {
            System.out.printf("Drive progress: %.1f%% complete (%.2fm of %.2fm)\n", 
                             percentComplete * 100.0, 
                             averageDistance * WHEEL_CIRCUMFERENCE,
                             m_distance);
        }
        
        // Check if we've reached target distance
        if (Math.abs(averageDistance) >= Math.abs(m_targetEncoderDistance) - ENCODER_TOLERANCE) {
            System.out.println("Target reached! Stopping at " + 
                              (averageDistance * WHEEL_CIRCUMFERENCE) + "m");
            m_isFinished = true;
        }
    }
    
    /**
     * Calculate motor power using a trapezoidal motion profile
     * 
     * @param elapsedSeconds Time elapsed since start
     * @param percentComplete Percentage of distance completed
     * @return Optimized power level
     */
    private double calculateProfiledPower(double elapsedSeconds, double percentComplete) {
        double power;
        
        // Acceleration phase - ramp up power
        if (elapsedSeconds < ACCELERATION_TIME) {
            // Linear ramp from MIN_POWER to maxPower
            power = MIN_POWER + (m_maxPower - MIN_POWER) * (elapsedSeconds / ACCELERATION_TIME);
        } 
        // Deceleration phase - slow down as we approach target
        else if (percentComplete > (1.0 - DECELERATION_THRESHOLD)) {
            // Calculate how far into deceleration zone we are (0 to 1)
            double decelerationProgress = 
                (percentComplete - (1.0 - DECELERATION_THRESHOLD)) / DECELERATION_THRESHOLD;
            
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
        long elapsedMillis = System.currentTimeMillis() - m_startTimeMillis;
        boolean timedOut = elapsedMillis >= (m_timeout * 1000);
        
        if (timedOut && !m_isFinished) {
            System.out.println("Drive command timed out after " + 
                               (elapsedMillis / 1000.0) + " seconds");
        }
        
        return m_isFinished || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop the motors
        m_drive.arcadeDrive(0, 0);
        
        if (interrupted) {
            System.out.println("Drive command interrupted");
        } else if (!m_isFinished) {
            System.out.println("Drive command timed out");
        } else {
            System.out.println("Drive command completed successfully");
        }
    }
}
