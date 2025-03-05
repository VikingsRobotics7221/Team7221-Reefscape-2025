// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_16to1_Straight.java

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer; // For precise timing
import frc.robot.subsystems.DriveSubsystem;

/**
 * HIGH-PRECISION AUTONOMOUS DRIVE COMMAND OPTIMIZED FOR 16:1 RATIO!
 * 
 * With our new 16:1 gearing, we get SICK pushing power but need more
 * precise control for autonomous routines. This command uses encoder
 * feedback with closed-loop control to drive SUPER accurately!
 * 
 * coded by paysean
 */
public class Drivetrain_16to1_Straight extends Command {
    // Configuration Constants
    private static final double CIRCUMFERENCE = Constants.WHEEL_DIAMETER * Math.PI;
    private static final double RAMP_TIME = 0.5; // seconds to reach full speed (smoother with 16:1)
    
    // Instance Variables
    private final DriveSubsystem drivetrain = Robot.m_driveSubsystem;
    private final double forwardPower;
    private final double goalDistance;
    private final double timeout;
    
    // Tracking variables
    private double startTime;
    private double leftStartPos, rightStartPos;
    private boolean hasLoggedCompletion = false;
    
    /**
     * Creates a high-precision drive command for 16:1 drivetrain!
     * 
     * @param distance Distance to drive in meters
     * @param power Power level (0 to 1.0)
     * @param timeoutSeconds Maximum time to run (seconds)
     */
    public Drivetrain_16to1_Straight(double distance, double power, double timeoutSeconds) {
        forwardPower = power;
        timeout = timeoutSeconds;
        
        // Convert distance to rotations for encoder tracking
        goalDistance = distance / CIRCUMFERENCE;

        addRequirements(drivetrain);
        
        System.out.println(">> CREATING 16:1 HIGH-PRECISION DRIVE - DISTANCE: " + 
                          distance + "m, POWER: " + power);
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public Drivetrain_16to1_Straight(double distance, double power) {
        this(distance, power, 10.0); // Default 10 second timeout
    }

    @Override
    public void initialize() {
        System.out.println(">> STARTING 16:1 PRECISION DRIVE!");
        drivetrain.arcadeDrive(0, 0);
        
        // Record encoder starting positions
        leftStartPos = drivetrain.getLeftPosition();
        rightStartPos = drivetrain.getRightPosition();
        
        // Record start time for ramping and timeout
        startTime = Timer.getFPGATimestamp();
        
        // Reset tracking variables
        hasLoggedCompletion = false;
    }

    @Override
    public void execute() {
        // Get current distance traveled (average of both sides)
        double leftCurrentPos = drivetrain.getLeftPosition() - leftStartPos;
        double rightCurrentPos = drivetrain.getRightPosition() - rightStartPos;
        double avgDistance = (Math.abs(leftCurrentPos) + Math.abs(rightCurrentPos)) / 2.0;
        
        // Calculate power with ramping at start
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        double rampFactor = Math.min(elapsedTime / RAMP_TIME, 1.0);
        
        // Also ramp down as we approach target
        double distanceRemaining = goalDistance - avgDistance;
        double approachFactor = 1.0;
        
        // When we're within 20% of target, start ramping down
        if (distanceRemaining < goalDistance * 0.2) {
            approachFactor = distanceRemaining / (goalDistance * 0.2);
            approachFactor = Math.max(approachFactor, 0.3); // Don't go below 30%
        }
        
        // Calculate final power with direction
        double direction = Math.signum(goalDistance);
        double power = forwardPower * rampFactor * approachFactor * direction;
        
        // Apply slight correction if sides are uneven (since we don't have gyro)
        double powerDiff = (Math.abs(leftCurrentPos) - Math.abs(rightCurrentPos)) * 0.05;
        
        // Drive with correction
        if (powerDiff > 0) {
            // Left side ahead, slow it down slightly
            drivetrain.tankDrive(power * 0.95, power * 1.05);
        } else {
            // Right side ahead, slow it down slightly
            drivetrain.tankDrive(power * 1.05, power * 0.95);
        }
        
        // Log progress periodically
        if (elapsedTime > 0.5 && Math.floor(elapsedTime * 2) / 2.0 == elapsedTime) {
            double percentComplete = avgDistance / goalDistance * 100.0;
            System.out.printf(">> PRECISION DRIVE: %.1f%% complete (%.2fm of %.2fm)\n", 
                             percentComplete, 
                             avgDistance * CIRCUMFERENCE,
                             goalDistance * CIRCUMFERENCE);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Check distance completion
        double leftCurrentPos = drivetrain.getLeftPosition() - leftStartPos;
        double rightCurrentPos = drivetrain.getRightPosition() - rightStartPos;
        double avgDistance = (Math.abs(leftCurrentPos) + Math.abs(rightCurrentPos)) / 2.0;
        
        // Check timeout
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        boolean timeoutExpired = elapsedTime >= timeout;
        
        // Check if we've reached target distance
        boolean distanceReached = avgDistance >= Math.abs(goalDistance);
        
        // Log completion only once
        if ((distanceReached || timeoutExpired) && !hasLoggedCompletion) {
            hasLoggedCompletion = true;
            
            if (timeoutExpired) {
                System.out.println(">> PRECISION DRIVE TIMED OUT after " + elapsedTime + 
                                  " seconds! Reached " + (avgDistance / Math.abs(goalDistance) * 100) + "%");
            } else {
                System.out.println(">> PRECISION DRIVE COMPLETE! Traveled " + 
                                  (avgDistance * CIRCUMFERENCE) + " meters");
            }
        }
        
        return distanceReached || timeoutExpired;
    }

    @Override
    public void end(boolean interrupted) {
        // Safety first - stop motors
        drivetrain.arcadeDrive(0, 0);
        
        if (interrupted) {
            System.out.println(">> PRECISION DRIVE INTERRUPTED!");
        }
    }
}
