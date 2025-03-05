// Author: Team 7221
// Last Updated: March 2025

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.subsystems.DriveSubsystem;

/**
 * Drivetrain_GyroStraight
 * 
 * TIME-BASED DRIVE STRAIGHT COMMAND (NO GYRO NEEDED!)
 * Makes the robot drive forward a specific distance using encoder feedback
 * without relying on a gyro for straight line correction.
 * 
 * coded by paysean
 */
public class Drivetrain_GyroStraight extends Command {
	// Configuration Constants
	private static final double CIRCUMFRENCE = Constants.WHEEL_DIAMETER * Math.PI;
	
	// Instance Variables
	DriveSubsystem drivetrain = Robot.m_driveSubsystem;
	double forwardPower, goalDistance;
	
	// Variables for time-based driving
	private long startTime;
	
	/**
	 * Creates a command that drives straight without gyro!
	 * Uses encoder feedback to go the right distance.
	 * 
	 * @param distance Distance to drive in meters
	 * @param power Power level (0 to 1.0)
	 */
	public Drivetrain_GyroStraight(double distance, double power) {
		forwardPower = power;
		
		// convert distance to revolutions
		goalDistance = distance / CIRCUMFRENCE;

		addRequirements(drivetrain);
		
		System.out.println(">> CREATING DRIVE STRAIGHT COMMAND - DISTANCE: " + 
		                  distance + "m, POWER: " + power);
	}
	
	@Override
	public void initialize() {
		System.out.println(">> STARTING DRIVE STRAIGHT!");
		drivetrain.arcadeDrive(0, 0);
		drivetrain.resetEncoders();
		startTime = System.currentTimeMillis();
	}

	@Override
	public void execute() {
		// Just drive straight with no correction!
		// Since we don't have a gyro, we're relying on balanced motor power
		drivetrain.arcadeDrive(forwardPower, 0);
		
		// Debug info
		if (System.currentTimeMillis() - startTime > 500 && 
		   (System.currentTimeMillis() - startTime) % 500 < 20) {
			System.out.println(">> DRIVING: " + 
			                  (Math.abs(drivetrain.getLeftFrontPosition()) / goalDistance * 100) + 
			                  "% complete");
		}
	}
	
	@Override
	public boolean isFinished() {
		boolean leftFrontGoalReached = Math.abs(drivetrain.getLeftFrontPosition()) >= goalDistance;
		boolean rightFrontGoalReached = Math.abs(drivetrain.getRightFrontPosition()) >= goalDistance;
		
		// We're done when either side reaches the goal
		if (leftFrontGoalReached || rightFrontGoalReached) {
			System.out.println(">> DRIVE STRAIGHT COMPLETE!");
			return true;
		}
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.arcadeDrive(0, 0);
		if (interrupted) {
			System.out.println(">> DRIVE STRAIGHT INTERRUPTED!");
		}
	}
}
