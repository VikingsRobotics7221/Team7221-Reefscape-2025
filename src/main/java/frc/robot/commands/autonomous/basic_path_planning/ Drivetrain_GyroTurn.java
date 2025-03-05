// Author: Team 7221
// Last Updated: March 2025

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.subsystems.DriveSubsystem;

/** 
 * Drivetrain_GyroTurn
 * 
 * TIME-BASED TURN COMMAND (NO GYRO NEEDED!)
 * Makes the robot turn in place using a time-based approach
 * since we don't have a gyro to measure our angle.
 * 
 * It uses a simple time calculation to estimate when we've turned enough.
 * 
 * coded by paysean
 */
public class Drivetrain_GyroTurn extends Command {
	// Configuration Constants
	private static final double MAX_POWER = Constants.MAX_POWER_GYRO;
	
	// Time-based turning constants
	private static final double DEGREES_PER_SECOND_AT_FULL_POWER = 120.0; // ~120° per second at full power
	
	// Instance Variables
	DriveSubsystem drivetrain = Robot.m_driveSubsystem;
	double goalAngle;
	private long startTime;
	private long turnDurationMs;
	
	/** 
	 * Creates a turn command that works without a gyro!
	 * Uses time-based estimation to turn approximately the right amount.
	 * 
	 * @param angle Angle to turn in degrees (positive = clockwise)
	 */
	public Drivetrain_GyroTurn(double angle) {
		goalAngle = angle;
		addRequirements(drivetrain);
		
		System.out.println(">> CREATING TURN COMMAND - TARGET: " + angle + "°");
	}
	
	@Override
	public void initialize() {
		System.out.println(">> STARTING TURN: " + goalAngle + "°");
		drivetrain.arcadeDrive(0, 0);
		startTime = System.currentTimeMillis();
		
		// Calculate how long we need to turn based on the angle
		// Adjust power based on turn size
		double turnPower = MAX_POWER;
		if (Math.abs(goalAngle) < 45) {
			// Use lower power for small turns
			turnPower = MAX_POWER * 0.7;
		}
		
		// Calculate turn duration
		double degreesPerSecond = DEGREES_PER_SECOND_AT_FULL_POWER * turnPower;
		turnDurationMs = (long)(Math.abs(goalAngle) / degreesPerSecond * 1000);
		
		System.out.println(">> ESTIMATED TURN TIME: " + turnDurationMs + "ms");
	}

	@Override
	public void execute() {
		// Calculate direction of turn
		double turnDirection = Math.signum(goalAngle);
		
		// Apply turn power
		drivetrain.arcadeDrive(0, -turnDirection * MAX_POWER);
		
		// Debug info
		if (System.currentTimeMillis() - startTime > 250 && 
		    (System.currentTimeMillis() - startTime) % 250 < 20) {
			
			long elapsed = System.currentTimeMillis() - startTime;
			int percentComplete = (int)(elapsed * 100 / turnDurationMs);
			System.out.println(">> TURNING: " + percentComplete + "% complete");
		}
	}
	
	@Override
	public boolean isFinished() {
		// We're done when we've turned for the calculated amount of time
		boolean timeComplete = (System.currentTimeMillis() - startTime) >= turnDurationMs;
		
		if (timeComplete) {
			System.out.println(">> TURN COMPLETE!");
		}
		
		return timeComplete;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.arcadeDrive(0, 0);
		
		if (interrupted) {
			System.out.println(">> TURN INTERRUPTED!");
		}
	}
}
