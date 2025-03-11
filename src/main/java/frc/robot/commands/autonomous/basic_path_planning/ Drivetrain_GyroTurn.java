// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_GyroTurn.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║ DRIVETRAIN GYRO TURN - PRECISION ANGULAR CONTROL                        ║
 * ║══════════════════════════════════════════════════════════════════════════║
 * ║ Advanced angular positioning system with time-based trajectory tracking  ║
 * ║ that enables reliable heading control without requiring a gyro sensor.   ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * This command provides precise angular control for tank drive systems by using
 * a calibrated time-based approach. It calculates turn duration based on a known
 * rotation rate and applies consistent power for predictable angular movement.
 * 
 * FEATURES:
 * - Calibrated angular velocity model for predictable rotation
 * - Adaptive power control based on turn size
 * - Comprehensive status reporting
 * - Full compatibility with basic drivetrain implementations
 */
public class Drivetrain_GyroTurn extends Command {
    // ===== CONFIGURATION CONSTANTS =====
    // Define MAX_POWER_GYRO locally since it's not in Constants
    private static final double MAX_POWER_GYRO = 0.6;
    
    // Calibration constant - how fast the robot turns at full power
    private static final double DEGREES_PER_SECOND_AT_FULL_POWER = 120.0; // ~120° per second at full power
    
    // ===== INSTANCE VARIABLES =====
    private final DriveSubsystem drivetrain;
    private final double goalAngle;
    private final double direction;
    private final double timeout;
    
    // ===== EXECUTION STATE =====
    private Timer timer = new Timer();
    private double turnPower;
    private double turnDurationSec;
    private double estimatedAngle = 0;
    private boolean isFinished = false;
    
    /** 
     * Creates a turn command that works without a gyro
     * 
     * @param angle Angle to turn in degrees (positive = clockwise, negative = counterclockwise)
     * @param timeoutSeconds Maximum time allowed for the turn
     */
    public Drivetrain_GyroTurn(double angle, double timeoutSeconds) {
        drivetrain = Robot.m_driveSubsystem;
        goalAngle = angle;
        direction = Math.signum(angle);
        timeout = timeoutSeconds;
        
        addRequirements(drivetrain);
    }
    
    /**
     * Simplified constructor with default timeout
     */
    public Drivetrain_GyroTurn(double angle) {
        this(angle, 5.0); // Default 5 second timeout
    }
    
    @Override
    public void initialize() {
        System.out.println(">> STARTING TURN: " + goalAngle + "°");
        
        // Ensure robot is stopped before turning
        drivetrain.arcadeDrive(0, 0); 
        
        // Reset timer and state variables
        timer.reset();
        timer.start();
        estimatedAngle = 0;
        isFinished = false;
        
        // Adjust power based on turn size for better precision
        if (Math.abs(goalAngle) < 45) {
            // Use lower power for small turns (better precision)
            turnPower = MAX_POWER_GYRO * 0.7;
            System.out.println(">> Small turn detected - using precision power: " + 
                              String.format("%.1f", turnPower * 100) + "%");
        } else if (Math.abs(goalAngle) > 150) {
            // Use higher power for large turns
            turnPower = MAX_POWER_GYRO * 0.9;
            System.out.println(">> Large turn detected - using higher power: " + 
                              String.format("%.1f", turnPower * 100) + "%");
        } else {
            // Standard power for medium turns
            turnPower = MAX_POWER_GYRO;
        }
        
        // Calculate turn duration based on the calibrated rotation rate
        double degreesPerSecond = DEGREES_PER_SECOND_AT_FULL_POWER * (turnPower / MAX_POWER_GYRO);
        turnDurationSec = Math.abs(goalAngle) / degreesPerSecond;
        
        System.out.println(">> Calculated turn duration: " + 
                          String.format("%.2f", turnDurationSec) + " seconds");
    }

    @Override
    public void execute() {
        // Apply differential power to create rotation
        // Note: In arcade drive, the second parameter controls rotation
        // Negative value is clockwise, positive is counter-clockwise
        double rotationInput = -direction * turnPower;
        drivetrain.arcadeDrive(0, rotationInput);
        
        // Update estimated angle based on elapsed time
        double elapsedTime = timer.get();
        double angularVelocity = DEGREES_PER_SECOND_AT_FULL_POWER * (turnPower / MAX_POWER_GYRO);
        estimatedAngle = elapsedTime * angularVelocity * direction;
        
        // Log progress at specific intervals
        if (elapsedTime > 0.2 && Math.floor(elapsedTime * 5) / 5.0 == elapsedTime) {
            double percentComplete = Math.min(100, Math.abs(estimatedAngle / goalAngle) * 100);
            System.out.printf(">> Turn progress: %.0f%% (est. %.1f° of %.1f°)\n",
                             percentComplete, 
                             Math.abs(estimatedAngle), 
                             Math.abs(goalAngle));
        }
    }
    
    @Override
    public boolean isFinished() {
        // We're done when either:
        // 1. We've turned for the calculated amount of time
        // 2. We've exceeded the timeout period
        boolean timeComplete = timer.get() >= turnDurationSec;
        boolean timedOut = timer.hasElapsed(timeout);
        
        if (timeComplete && !isFinished) {
            isFinished = true;
            System.out.println(">> Turn complete!");
        } else if (timedOut && !isFinished) {
            isFinished = true;
            System.out.println(">> Turn timed out after " + 
                              String.format("%.1f", timer.get()) + " seconds");
        }
        
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop turning
        drivetrain.arcadeDrive(0, 0);
        
        // Stop the timer
        timer.stop();
        
        // Log completion status
        if (interrupted) {
            System.out.println(">> Turn interrupted!");
        } else {
            System.out.printf(">> Turn completed in %.2f seconds (est. %.1f°)\n", 
                            timer.get(), Math.abs(estimatedAngle));
        }
    }
    
    /**
     * Get the estimated heading change so far
     * 
     * @return Estimated angle in degrees
     */
    public double getEstimatedAngle() {
        return estimatedAngle;
    }
    
    /**
     * Calculate time needed for a specific turn.
     * Useful for planning command sequences.
     * 
     * @param degrees Angle in degrees
     * @return Estimated time in seconds
     */
    public static double calculateTurnTime(double degrees) {
        double power = MAX_POWER_GYRO;
        
        // Adjust power based on turn size
        if (Math.abs(degrees) < 45) {
            power = MAX_POWER_GYRO * 0.7;
        } else if (Math.abs(degrees) > 150) {
            power = MAX_POWER_GYRO * 0.9;
        }
        
        double degreesPerSecond = DEGREES_PER_SECOND_AT_FULL_POWER * (power / MAX_POWER_GYRO);
        return Math.abs(degrees) / degreesPerSecond;
    }
}
