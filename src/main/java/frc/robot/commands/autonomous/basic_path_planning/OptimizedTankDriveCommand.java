// src/main/java/frc/robot/commands/autonomous/basic_path_planning/OptimizedTankDriveCommand.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * OptimizedTankDriveCommand - SUPER PRECISE MOVEMENT FOR 16:1 RATIO!
 * 
 * This command provides smooth acceleration, deceleration, and turn profiling
 * specifically optimized for our 16:1 gear ratio tank drive.
 * 
 * Advantages over regular drive commands:
 * 1. Adjusts power curves for the higher torque of 16:1 ratio
 * 2. Uses separate acceleration and deceleration profiles
 * 3. Implements encoder-based closed-loop control
 * 4. Provides predictive stopping to prevent overshooting
 * 
 * coded by paysean
 */
public class OptimizedTankDriveCommand extends Command {
    
    // ========= CONSTANTS FOR MOTION PROFILING =========
    // These are specially tuned for 16:1 ratio!
    private static final double ACCELERATION_TIME = 0.4; // seconds to reach full speed
    private static final double DECELERATION_THRESHOLD = 0.4; // portion of distance where we start slowing
    private static final double MIN_POWER = 0.07; // minimum power to overcome static friction
    private static final double ENCODER_TOLERANCE = 0.05; // rotations of tolerance
    
    // ========= TRACKING VARIABLES =========
    private final DriveSubsystem m_drive;
    private final double m_distance; // in meters
    private final double m_maxPower; // 0 to 1
    private final double m_timeout; // in seconds
    
    private double m_startingLeftPosition;
    private double m_startingRightPosition;
    private double m_targetEncoderDistance; // in rotations
    private long m_startTimeMillis;
    private boolean m_isFinished = false;
    
    // ASCII ART BECAUSE WHY NOT?!
    //   _______
    //  |       |
    //  | DRIVE |
    //  |_______|
    // _/       \_
    // |         |
    // |  16:1   |
    // |  POWER  |
    // |_________|
    //    o   o
    
    /**
     * Creates a precisely tuned drive command for our 16:1 ratio!
     * 
     * @param distance Distance to travel in meters (+ = forward, - = backward)
     * @param maxPower Maximum power level (0 to 1)
     * @param timeoutSeconds Maximum time allowed for the operation
     */
    public OptimizedTankDriveCommand(double distance, double maxPower, double timeoutSeconds) {
        m_drive = Robot.m_driveSubsystem;
        m_distance = distance;
        m_maxPower = Math.abs(maxPower);
        m_timeout = timeoutSeconds;
        
        // Calculate target encoder distance (rotations)
        m_targetEncoderDistance = distance / (Math.PI * Constants.WHEEL_DIAMETER);
        
        // We need control of the drivetrain
        addRequirements(m_drive);
        
        System.out.println(">> CREATING 16:1 OPTIMIZED DRIVE COMMAND!");
        System.out.println(">> DISTANCE: " + distance + "m, MAX POWER: " + maxPower);
        System.out.println(">> TARGET ENCODER: " + m_targetEncoderDistance + " rotations");
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public OptimizedTankDriveCommand(double distance, double maxPower) {
        this(distance, maxPower, 10.0); // Default 10 second timeout
    }
    
    @Override
    public void initialize() {
        System.out.println("");
        System.out.println(">> STARTING 16:1 OPTIMIZED DRIVE: " + m_distance + "m");
        
        // Stop any current movement
        m_drive.arcadeDrive(0, 0);
        
        // Record starting positions for tracking
        m_startingLeftPosition = m_drive.getLeftPosition();
        m_startingRightPosition = m_drive.getRightPosition();
        
        // Record start time for timeout and profiling
        m_startTimeMillis = System.currentTimeMillis();
        
        // Reset finished flag
        m_isFinished = false;
        
        System.out.println(">> INITIAL POSITIONS - LEFT: " + m_startingLeftPosition + 
                          ", RIGHT: " + m_startingRightPosition);
    }
    
    @Override
    public void execute() {
        // Calculate how long we've been running
        double elapsedSeconds = (System.currentTimeMillis() - m_startTimeMillis) / 1000.0;
        
        // Get current positions and calculate distance traveled
        double currentLeftPosition = m_drive.getLeftPosition() - m_startingLeftPosition;
        double currentRightPosition = m_drive.getRightPosition() - m_startingRightPosition;
        
        // Average distance traveled (taking direction into account)
        double direction = Math.signum(m_targetEncoderDistance);
        double averageDistance = direction * (Math.abs(currentLeftPosition) + 
                                            Math.abs(currentRightPosition)) / 2.0;
        
        // Calculate percentage of journey completed
        double percentComplete = Math.abs(averageDistance / m_targetEncoderDistance);
        
        // Calculate power based on acceleration and deceleration profiles
        double power = calculateProfiledPower(elapsedSeconds, percentComplete);
        
        // Apply direction
        power *= direction;
        
        // Apply small correction if left and right are uneven
        double leftAdjust = 1.0;
        double rightAdjust = 1.0;
        
        // If we detect uneven movement (more than 2% difference), apply correction
        double leftDistance = Math.abs(currentLeftPosition);
        double rightDistance = Math.abs(currentRightPosition);
        
        if (leftDistance > rightDistance * 1.02) {
            // Left side is moving faster, slow it slightly
            leftAdjust = 0.95;
            rightAdjust = 1.05;
        } else if (rightDistance > leftDistance * 1.02) {
            // Right side is moving faster, slow it slightly
            leftAdjust = 1.05;
            rightAdjust = 0.95;
        }
        
        // Drive with balanced power
        m_drive.tankDrive(power * leftAdjust, power * rightAdjust);
        
        // Log progress occasionally
        if (elapsedSeconds > 0.5 && Math.floor(elapsedSeconds * 2) / 2.0 == elapsedSeconds) {
            System.out.printf(">> OPTIMIZED DRIVE: %.1f%% complete (%.2fm of %.2fm)\n", 
                             percentComplete * 100.0, 
                             averageDistance * Math.PI * Constants.WHEEL_DIAMETER,
                             m_distance);
        }
        
        // Check if we've reached target distance
        if (Math.abs(averageDistance) >= Math.abs(m_targetEncoderDistance) - ENCODER_TOLERANCE) {
            System.out.println(">> TARGET REACHED! Stopping at " + 
                              (averageDistance * Math.PI * Constants.WHEEL_DIAMETER) + "m");
            m_isFinished = true;
        }
    }
    
    /**
     * Calculate power using a trapezoidal motion profile
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
            
            // Deceleration curve - quadratic for smoother stopping with 16:1 ratio
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
            System.out.println(">> OPTIMIZED DRIVE TIMED OUT after " + 
                               (elapsedMillis / 1000.0) + " seconds!");
        }
        
        return m_isFinished || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop the motors
        m_drive.arcadeDrive(0, 0);
        
        if (interrupted) {
            System.out.println(">> OPTIMIZED DRIVE INTERRUPTED!");
        } else if (!m_isFinished) {
            System.out.println(">> OPTIMIZED DRIVE TIMED OUT!");
        } else {
            System.out.println(">> OPTIMIZED DRIVE COMPLETED SUCCESSFULLY!");
        }
    }
}
