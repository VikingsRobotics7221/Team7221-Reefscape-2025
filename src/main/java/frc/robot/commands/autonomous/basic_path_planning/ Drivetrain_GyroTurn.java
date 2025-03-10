// Author: Team 7221
// Last Updated: March 2025

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.subsystems.DriveSubsystem;

/** 
 * Drivetrain_GyroTurn - Time-based Turn Command
 * 
 * This command enables precise angular control without requiring a gyro.
 * It uses a calibrated time-based approach to achieve predictable turning.
 * 
 * How it works:
 * 1. Takes a target angle as input
 * 2. Calculates how long to apply turning power based on a calibrated degrees/second rate
 * 3. Applies consistent turning power for that duration
 * 4. Completes when the calculated time has elapsed
 * 
 * System Integration:
 * - Requires the DriveSubsystem defined in Robot.java
 * - Uses power constants from Constants.java
 * - Can be used in autonomous command sequences
 */
public class Drivetrain_GyroTurn extends Command {
    // Configuration Constants
    private static final double MAX_POWER = Constants.MAX_POWER_GYRO;
    private static final double DEGREES_PER_SECOND_AT_FULL_POWER = 120.0; // ~120° per second at full power
    
    // Instance Variables
    private final DriveSubsystem drivetrain;
    private final double goalAngle;
    private long startTime;
    private long turnDurationMs;
    
    /** 
     * Creates a turn command that works without a gyro
     * 
     * @param angle Angle to turn in degrees (positive = clockwise, negative = counterclockwise)
     */
    public Drivetrain_GyroTurn(double angle) {
        drivetrain = Robot.m_driveSubsystem;
        goalAngle = angle;
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        System.out.println("Starting turn: " + goalAngle + "°");
        drivetrain.arcadeDrive(0, 0); // Ensure robot is stopped before turning
        startTime = System.currentTimeMillis();
        
        // Calculate turn duration based on the angle
        // Adjust power based on turn size for better precision
        double turnPower = MAX_POWER;
        if (Math.abs(goalAngle) < 45) {
            // Use lower power for small turns
            turnPower = MAX_POWER * 0.7;
        }
        
        // Calculate turn duration
        double degreesPerSecond = DEGREES_PER_SECOND_AT_FULL_POWER * turnPower;
        turnDurationMs = (long)(Math.abs(goalAngle) / degreesPerSecond * 1000);
        
        System.out.println("Estimated turn time: " + turnDurationMs + "ms");
    }

    @Override
    public void execute() {
        // Calculate direction of turn
        double turnDirection = Math.signum(goalAngle);
        
        // Apply turn power
        drivetrain.arcadeDrive(0, -turnDirection * MAX_POWER);
        
        // Provide periodic progress updates
        long elapsed = System.currentTimeMillis() - startTime;
        if (elapsed > 250 && elapsed % 250 < 20) {
            int percentComplete = (int)(elapsed * 100 / turnDurationMs);
            System.out.println("Turn progress: " + percentComplete + "%");
        }
    }
    
    @Override
    public boolean isFinished() {
        // We're done when we've turned for the calculated amount of time
        boolean timeComplete = (System.currentTimeMillis() - startTime) >= turnDurationMs;
        
        if (timeComplete) {
            System.out.println("Turn complete!");
        }
        
        return timeComplete;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0); // Stop turning
        
        if (interrupted) {
            System.out.println("Turn interrupted!");
        }
    }
}
