// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_GyroStrafe.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * ╔═════════════════════════════════════════════════════════════════╗
 * ║  TANK DRIVE STRAFE SIMULATION                                  ║
 * ║  "The Impossible Made Possible"                                ║
 * ╚═════════════════════════════════════════════════════════════════╝
 * 
 * This command creates a "strafe-like" motion on a tank drive by applying
 * opposite power to the tracks. Since tank drives cannot physically strafe,
 * this creates a rotation-based movement that approximates lateral motion.
 * 
 * The tank drive "strafe" works by:
 * 1. Applying opposite power to each track (one forward, one reverse)
 * 2. Creating a rotation in place that shifts the robot sideways
 * 3. Using carefully timed power application to achieve desired displacement
 * 
 * This implementation uses time-based tracking instead of encoders for
 * compatibility with basic drivetrains without position feedback.
 */
public class Drivetrain_GyroStrafe extends Command {
  
    // ===== PHYSICS CONSTANTS =====
    private static final double WHEEL_CIRCUMFERENCE = Constants.Dimensions.WHEEL_DIAMETER * Math.PI;
    
    // ===== CALIBRATION CONSTANTS =====
    // How much "strafe" distance is achieved per second at full power
    private static final double STRAFE_RATE_METERS_PER_SECOND = 0.5;
    
    // How efficient the strafe motion is compared to direct travel
    private static final double STRAFE_EFFICIENCY_FACTOR = 1.8;
    
    // ===== COMMAND STATE VARIABLES =====
    private final DriveSubsystem drivetrain;
    private final double strafePower;     // Power level (0-1.0)
    private final double goalDistance;    // Target "strafe" distance in meters
    private final double direction;       // Direction: positive = right, negative = left
    private final double timeout;         // Maximum execution time
    
    // ===== EXECUTION STATE =====
    private Timer timer = new Timer();
    private double estimatedDistanceTraveled = 0.0;
    private boolean isFinished = false;
    
    /**
     * Creates a new simulated strafe command for tank drive.
     * 
     * @param distance Distance to "strafe" in meters (positive = right, negative = left)
     * @param power Power level for the motion (0.0 to 1.0)
     * @param timeoutSeconds Maximum time allowed for execution
     */
    public Drivetrain_GyroStrafe(double distance, double power, double timeoutSeconds) {
        // Store direction separately from magnitude
        this.direction = Math.signum(distance);
        
        // Store power as a positive value - direction is handled separately
        this.strafePower = Math.abs(power);
        
        // Store the target distance (absolute value)
        this.goalDistance = Math.abs(distance);
        
        // Store maximum execution time
        this.timeout = timeoutSeconds;
        
        // Get reference to drivetrain
        this.drivetrain = Robot.m_driveSubsystem;
        
        // Register the drivetrain requirement
        addRequirements(drivetrain);
        
        // Log creation information
        System.out.println(">> Creating strafe simulation - Distance: " + 
                          distance + "m, Power: " + power + 
                          ", Timeout: " + timeoutSeconds + "s");
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public Drivetrain_GyroStrafe(double distance, double power) {
        this(distance, power, 5.0); // Default 5 second timeout
    }

    @Override
    public void initialize() {
        System.out.println(">> Starting tank drive strafe simulation");
        System.out.println(">> Direction: " + (direction > 0 ? "RIGHT" : "LEFT"));
        
        // Stop any existing movement
        drivetrain.arcadeDrive(0, 0);
        
        // Reset timer and tracking variables
        timer.reset();
        timer.start();
        estimatedDistanceTraveled = 0.0;
        isFinished = false;
    }

    @Override
    public void execute() {
        // Calculate power for each side (opposite values to create rotation)
        double leftPower = strafePower * direction;
        double rightPower = -strafePower * direction;
        
        // Apply tank drive with differential powers
        drivetrain.tankDrive(leftPower, rightPower);
        
        // Calculate elapsed time
        double elapsedTime = timer.get();
        
        // Estimate distance traveled based on time and power
        // This replaces encoder feedback which is not available
        double distancePerSecond = STRAFE_RATE_METERS_PER_SECOND * strafePower;
        estimatedDistanceTraveled = elapsedTime * distancePerSecond;
        
        // Provide periodic progress updates (every 0.5 seconds)
        if (elapsedTime > 0.5 && (Math.floor(elapsedTime * 2) / 2.0 == elapsedTime)) {
            // Calculate progress percentage
            double percentComplete = estimatedDistanceTraveled / goalDistance * 100;
            
            System.out.printf(">> Strafing: %.1f%% complete (est. %.2fm of %.2fm)\n", 
                              percentComplete, 
                              estimatedDistanceTraveled, 
                              goalDistance);
        }
        
        // Check if we've reached the goal distance
        if (estimatedDistanceTraveled >= goalDistance) {
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Command is complete when we've traveled the goal distance or timed out
        return isFinished || timer.hasElapsed(timeout);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motors for safety
        drivetrain.arcadeDrive(0, 0);
        
        // Stop the timer
        timer.stop();
        
        // Report completion status
        if (interrupted) {
            System.out.println(">> Strafe interrupted!");
        } else {
            // Report completion and execution time
            double executionTime = timer.get();
            
            System.out.printf(">> Strafe completed in %.1f seconds (est. traveled %.2fm)\n", 
                           executionTime, estimatedDistanceTraveled);
            
            // If we timed out but didn't finish, report that
            if (!isFinished && timer.hasElapsed(timeout)) {
                System.out.println(">> Note: Strafe timed out before completion");
            }
        }
    }
    
    /**
     * Get estimated distance traveled in the strafe direction
     * 
     * @return Estimated travel distance in meters
     */
    public double getEstimatedDistance() {
        return estimatedDistanceTraveled;
    }
    
    /*
     * NOTE: This is how the command would be implemented with encoder support.
     * This code is commented out because the current DriveSubsystem does not
     * have encoder methods implemented. When encoders are added, this implementation
     * could be used instead of the time-based estimation above.
     *
     * // Position tracking with encoders
     * private double leftEncoderStart, rightEncoderStart;
     *
     * @Override
     * public void initialize() {
     *     // Record starting encoder positions
     *     leftEncoderStart = drivetrain.getLeftPosition();
     *     rightEncoderStart = drivetrain.getRightPosition();
     * }
     *
     * @Override
     * public boolean isFinished() {
     *     // Calculate distance traveled by each side
     *     double leftTravel = Math.abs(drivetrain.getLeftPosition() - leftEncoderStart);
     *     double rightTravel = Math.abs(drivetrain.getRightPosition() - rightEncoderStart);
     *     
     *     // Use the average travel distance to determine completion
     *     double avgTravel = (leftTravel + rightTravel) / 2.0;
     *     
     *     // Convert rotations to distance and account for efficiency factor
     *     double distance = avgTravel * WHEEL_CIRCUMFERENCE / STRAFE_EFFICIENCY_FACTOR;
     *     
     *     // Command is complete when we've traveled the goal distance
     *     return distance >= goalDistance || timer.hasElapsed(timeout);
     * }
     */
}
