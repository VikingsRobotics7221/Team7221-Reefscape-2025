// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_GyroStraight.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Precision autonomous straight-line driving command optimized for 16:1 gear ratio.
 * 
 * This command enables the robot to drive a specific distance in a straight line
 * using encoder feedback to maintain accuracy. The command carefully manages
 * acceleration and deceleration to prevent wheel slip and ensure precision.
 * 
 * How this connects to other components:
 * - Uses DriveSubsystem from Robot.java for motor control
 * - References Constants.java for wheel dimensions and control parameters
 * - Compatible with command groups in autonomous routines
 * 
 * Usage example:
 *   // Drive forward 2 meters at 60% power with 5 second timeout
 *   new Drivetrain_GyroStraight(2.0, 0.6, 5.0)
 */
public class Drivetrain_GyroStraight extends Command {
    
    // Physical constants
    private static final double WHEEL_CIRCUMFERENCE = Constants.Dimensions.WHEEL_DIAMETER * Math.PI;
    
    // Control parameters for smoother motion
    private static final double ACCELERATION_TIME = 0.5;  // Seconds to reach full speed
    private static final double DECELERATION_DISTANCE = 0.3;  // Portion of path where we slow down
    private static final double MIN_POWER = 0.1;  // Minimum power to overcome friction
    
    // System references and command parameters
    private final DriveSubsystem drivetrain;
    private final double distance;  // Target distance in meters
    private final double maxPower;  // Maximum power level (0-1.0)
    private final double timeout;   // Maximum time allowed for movement
    
    // State tracking variables
    private double startPosition;
    private double targetRotations;
    private Timer commandTimer = new Timer();
    private boolean isFinished = false;
    
    /**
     * Creates a command to drive straight for a specific distance.
     * 
     * @param distance Distance to travel in meters (positive = forward, negative = backward)
     * @param power Maximum power level (0.0 to 1.0)
     * @param timeoutSeconds Maximum time allowed to complete the movement
     */
    public Drivetrain_GyroStraight(double distance, double power, double timeoutSeconds) {
        this.drivetrain = Robot.m_driveSubsystem;
        this.distance = distance;
        this.maxPower = Math.abs(power);
        this.timeout = timeoutSeconds;
        
        // Convert target distance to wheel rotations
        this.targetRotations = Math.abs(distance) / WHEEL_CIRCUMFERENCE;
        
        // Register drivetrain requirement
        addRequirements(drivetrain);
        
        System.out.println("Drive straight command created: " + 
                           distance + "m at " + power + " power");
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public Drivetrain_GyroStraight(double distance, double power) {
        this(distance, power, 10.0);  // Default 10 second timeout
    }
    
    @Override
    public void initialize() {
        System.out.println("Starting drive straight: " + distance + "m");
        
        // Stop any existing movement
        drivetrain.arcadeDrive(0, 0);
        
        // Record starting position (average of left and right encoders)
        startPosition = (drivetrain.getLeftPosition() + drivetrain.getRightPosition()) / 2.0;
        
        // Reset and start the command timer
        commandTimer.reset();
        commandTimer.start();
        
        // Reset tracking variables
        isFinished = false;
    }
    
    @Override
    public void execute() {
        // Get current position
        double currentPosition = (drivetrain.getLeftPosition() + drivetrain.getRightPosition()) / 2.0;
        
        // Calculate how far we've traveled so far
        double traveledRotations = Math.abs(currentPosition - startPosition);
        
        // Calculate remaining distance
        double remainingRotations = targetRotations - traveledRotations;
        
        // Progress as a percentage
        double progress = traveledRotations / targetRotations;
        
        // Calculate appropriate power level
        double power = calculateSmoothPowerLevel(progress, remainingRotations);
        
        // Apply direction based on original distance parameter
        double directedPower = Math.signum(distance) * power;
        
        // Apply slight correction based on encoder difference to maintain straight line
        double leftPos = drivetrain.getLeftPosition();
        double rightPos = drivetrain.getRightPosition();
        double positionDifference = Math.abs(leftPos - rightPos) - 
                                  Math.abs(startPosition - startPosition);
        
        // Create a small turning correction if the robot is veering off course
        double turnCorrection = positionDifference * 0.1;
        turnCorrection = Math.max(-0.2, Math.min(0.2, turnCorrection));
        
        // Apply power with straight-line correction
        drivetrain.arcadeDrive(directedPower, turnCorrection);
        
        // Log progress occasionally (every second)
        double elapsedTime = commandTimer.get();
        if (elapsedTime > 1.0 && Math.floor(elapsedTime) == elapsedTime) {
            System.out.printf("Drive progress: %.1f%% (%.2fm of %.2fm)\n", 
                             progress * 100, 
                             traveledRotations * WHEEL_CIRCUMFERENCE,
                             targetRotations * WHEEL_CIRCUMFERENCE);
        }
        
        // Check if we've reached the target
        if (traveledRotations >= targetRotations) {
            isFinished = true;
        }
    }
    
    /**
     * Calculates a smooth power profile for consistent movement.
     * This creates acceleration at the start and deceleration near the end.
     * 
     * @param progress Current progress (0.0 to 1.0)
     * @param remaining Distance remaining in rotations
     * @return Appropriate power level (0.0 to maxPower)
     */
    private double calculateSmoothPowerLevel(double progress, double remaining) {
        double power;
        
        // Acceleration phase - first part of movement
        double elapsedTime = commandTimer.get();
        if (elapsedTime < ACCELERATION_TIME) {
            // Linear ramp from MIN_POWER to maxPower
            power = MIN_POWER + (maxPower - MIN_POWER) * (elapsedTime / ACCELERATION_TIME);
        }
        // Deceleration phase - last part of movement
        else if (progress > (1.0 - DECELERATION_DISTANCE)) {
            // Calculate deceleration factor based on remaining distance
            double decelerationFactor = remaining / (targetRotations * DECELERATION_DISTANCE);
            
            // Apply quadratic deceleration curve for smooth stopping
            power = MIN_POWER + (maxPower - MIN_POWER) * decelerationFactor * decelerationFactor;
        }
        // Cruise phase - middle part of movement
        else {
            power = maxPower;
        }
        
        return power;
    }
    
    @Override
    public boolean isFinished() {
        // Complete when we've either reached the target or timed out
        boolean timedOut = commandTimer.hasElapsed(timeout);
        
        if (timedOut && !isFinished) {
            System.out.println("Drive straight timed out after " + 
                              commandTimer.get() + " seconds");
        }
        
        return isFinished || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain for safety
        drivetrain.arcadeDrive(0, 0);
        
        // Stop the timer
        commandTimer.stop();
        
        // Log completion information
        if (interrupted) {
            System.out.println("Drive straight interrupted!");
        } else if (isFinished) {
            double actualDistance = Math.abs(
                (drivetrain.getLeftPosition() + drivetrain.getRightPosition()) / 2.0 - 
                startPosition) * WHEEL_CIRCUMFERENCE;
            
            System.out.printf("Drive straight completed: %.2fm in %.1f seconds\n",
                             actualDistance, commandTimer.get());
        }
    }
}
