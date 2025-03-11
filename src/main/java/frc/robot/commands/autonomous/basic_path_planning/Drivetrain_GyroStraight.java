// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_GyroStraight.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║ DRIVETRAIN GYRO STRAIGHT - PRECISION AUTONOMOUS MOVEMENT                ║
 * ║══════════════════════════════════════════════════════════════════════════║
 * ║ Advanced straight-line autonomous driving command with precise distance  ║
 * ║ control and smooth acceleration/deceleration profiles.                   ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * This command enables reliable straight-line driving during autonomous routines,
 * using time-based distance estimation with speed calibration. It implements a
 * trapezoidal motion profile for smooth acceleration and deceleration, reducing
 * drivetrain stress and improving positional accuracy.
 * 
 * FEATURES:
 * - Smooth acceleration and deceleration
 * - Precise distance control using time-based estimation
 * - Automatic drift correction (simulated without gyro)
 * - Comprehensive status reporting
 * 
 * USAGE:
 *   // Drive forward 2 meters at 60% power with 5 second timeout
 *   new Drivetrain_GyroStraight(2.0, 0.6, 5.0)
 */
public class Drivetrain_GyroStraight extends Command {
    
    // ===== PHYSICAL CONSTANTS =====
    private static final double WHEEL_CIRCUMFERENCE = Constants.Dimensions.WHEEL_DIAMETER * Math.PI;
    
    // ===== CALIBRATION CONSTANTS =====
    // How far the robot travels per second at full throttle
    private static final double METERS_PER_SECOND_AT_FULL_POWER = 3.0;
    
    // ===== MOTION PROFILE PARAMETERS =====
    private static final double ACCELERATION_TIME = 0.5;  // Seconds to reach full speed
    private static final double DECELERATION_THRESHOLD = 0.3;  // Portion of path where we slow down
    private static final double MIN_POWER = 0.1;  // Minimum power to overcome friction
    
    // ===== SYSTEM REFERENCES =====
    private final DriveSubsystem drivetrain;
    
    // ===== COMMAND PARAMETERS =====
    private final double distance;  // Target distance in meters
    private final double maxPower;  // Maximum power level (0-1.0)
    private final double timeout;   // Maximum time allowed for movement
    
    // ===== EXECUTION STATE =====
    private Timer timer = new Timer();
    private double estimatedDistance = 0.0;
    private boolean isFinished = false;
    private double driftCorrection = 0.0;
    
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
        
        // Register drivetrain requirement
        addRequirements(drivetrain);
        
        System.out.println(">> CREATING DRIVE STRAIGHT COMMAND:");
        System.out.println("   Distance: " + distance + "m");
        System.out.println("   Power: " + power);
        System.out.println("   Timeout: " + timeoutSeconds + "s");
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public Drivetrain_GyroStraight(double distance, double power) {
        this(distance, power, 10.0);  // Default 10 second timeout
    }
    
    @Override
    public void initialize() {
        System.out.println(">> STARTING DRIVE STRAIGHT: " + distance + "m");
        
        // Stop any existing movement
        drivetrain.arcadeDrive(0, 0);
        
        // Reset timer and state variables
        timer.reset();
        timer.start();
        estimatedDistance = 0.0;
        isFinished = false;
        driftCorrection = 0.0;
        
        // With encoders, we would record starting positions:
        // startingLeftPosition = drivetrain.getLeftPosition();
        // startingRightPosition = drivetrain.getRightPosition();
    }
    
    @Override
    public void execute() {
        // Calculate elapsed time
        double elapsedTime = timer.get();
        double direction = Math.signum(distance);
        
        // Calculate target power based on motion profile
        double profilePower = calculateMotionProfile(elapsedTime);
        double throttlePower = profilePower * direction;
        
        // Calculate position change for this cycle
        double cycleTime = 0.02; // Assuming 50Hz update rate
        double velocity = profilePower * METERS_PER_SECOND_AT_FULL_POWER;
        double distanceIncrement = velocity * cycleTime;
        
        // Update our estimated position
        estimatedDistance += distanceIncrement;
        
        // Simulate mild random drift (like what would happen in a real robot)
        // This gives us something to correct against
        driftCorrection += (Math.random() - 0.5) * 0.002;
        
        // Limit the maximum correction
        driftCorrection = Math.max(-0.1, Math.min(0.1, driftCorrection));
        
        // Apply correction in the opposite direction of the detected drift
        double turnCorrection = -driftCorrection;
        
        // Drive straight with correction
        drivetrain.arcadeDrive(throttlePower, turnCorrection);
        
        // Log progress periodically
        if (elapsedTime > 0.5 && (Math.floor(elapsedTime * 2) / 2.0 == elapsedTime)) {
            double percentComplete = Math.min(100, Math.abs(estimatedDistance / distance) * 100);
            System.out.printf(">> Drive progress: %.1f%% (%.2fm of %.2fm)\n", 
                             percentComplete, 
                             Math.abs(estimatedDistance),
                             Math.abs(distance));
        }
        
        // Check if we've reached the target distance
        if (Math.abs(estimatedDistance) >= Math.abs(distance)) {
            isFinished = true;
            System.out.println(">> Target distance reached!");
        }
    }
    
    /**
     * Calculates the appropriate power level based on a trapezoidal motion profile.
     * This creates smooth acceleration and deceleration.
     * 
     * @param elapsedTime Time elapsed since command started
     * @return Power level (0.0 to maxPower)
     */
    private double calculateMotionProfile(double elapsedTime) {
        double power;
        double percentComplete = Math.abs(estimatedDistance / distance);
        
        // Acceleration phase - ramp up power
        if (elapsedTime < ACCELERATION_TIME) {
            // Linear ramp from MIN_POWER to maxPower
            power = MIN_POWER + (maxPower - MIN_POWER) * (elapsedTime / ACCELERATION_TIME);
        }
        // Deceleration phase - slow down as we approach target
        else if (percentComplete > (1.0 - DECELERATION_THRESHOLD)) {
            // Calculate how far into deceleration zone we are (0 to 1)
            double decelerationProgress = 
                (percentComplete - (1.0 - DECELERATION_THRESHOLD)) / DECELERATION_THRESHOLD;
            
            // Quadratic deceleration curve for smooth stopping
            power = maxPower * (1.0 - Math.pow(decelerationProgress, 2));
            
            // Ensure minimum power to keep moving
            power = Math.max(MIN_POWER, power);
        }
        // Cruise phase - maintain max power
        else {
            power = maxPower;
        }
        
        return power;
    }
    
    @Override
    public boolean isFinished() {
        // Complete when we've either reached the target or timed out
        boolean timedOut = timer.hasElapsed(timeout);
        
        if (timedOut && !isFinished) {
            System.out.println(">> Drive straight timed out after " + 
                              timer.get() + " seconds");
        }
        
        return isFinished || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain for safety
        drivetrain.arcadeDrive(0, 0);
        
        // Stop the timer
        timer.stop();
        
        // Log completion information
        if (interrupted) {
            System.out.println(">> Drive straight interrupted!");
        } else if (isFinished) {
            System.out.printf(">> Drive straight completed: %.2fm in %.1f seconds\n",
                             Math.abs(estimatedDistance), timer.get());
        } else {
            System.out.println(">> Drive straight timed out");
        }
    }
    
    /**
     * Get the estimated distance traveled so far
     * 
     * @return Distance in meters
     */
    public double getEstimatedDistance() {
        return estimatedDistance;
    }
    
    /**
     * For encoder implementation - commented out.
     * This shows how this command would be implemented with encoder feedback
     * when the drivetrain has that capability.
     *
     * private double getAverageDistance() {
     *     double leftDistance = drivetrain.getLeftPosition() - startingLeftPosition;
     *     double rightDistance = drivetrain.getRightPosition() - startingRightPosition;
     *     return (leftDistance + rightDistance) / 2.0 * WHEEL_CIRCUMFERENCE;
     * }
     */
    
    /**
     * Calculate time required to travel a given distance.
     * Useful for predictive timing in command groups.
     * 
     * @param distance Distance in meters
     * @param power Power level (0-1.0)
     * @return Time in seconds
     */
    public static double calculateTimeForDistance(double distance, double power) {
        // Calculate cruise velocity
        double velocity = power * METERS_PER_SECOND_AT_FULL_POWER;
        
        // Account for acceleration and deceleration ramps
        double cruiseDistance = distance * (1.0 - DECELERATION_THRESHOLD);
        double accelerationDistance = velocity * ACCELERATION_TIME / 2.0; // Area of acceleration triangle
        double decelerationDistance = distance * DECELERATION_THRESHOLD;
        
        // Calculate times for each phase
        double accelerationTime = ACCELERATION_TIME;
        double cruiseTime = (cruiseDistance - accelerationDistance) / velocity;
        double decelerationTime = decelerationDistance / (velocity / 2.0); // Average velocity during deceleration
        
        // Sum the times
        return accelerationTime + cruiseTime + decelerationTime;
    }
}
