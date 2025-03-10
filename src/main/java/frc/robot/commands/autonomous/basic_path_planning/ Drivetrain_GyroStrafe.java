// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_GyroStrafe.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * Drivetrain_GyroStrafe - Tank Drive Strafe Simulation
 * 
 * This command creates a lateral "strafe-like" motion on a tank drive by applying
 * differential power to the tracks. Since tank drives cannot physically strafe,
 * this simulates the motion by rotating the tracks in opposite directions.
 * 
 * The command uses encoder feedback to determine when the robot has moved
 * the requested distance sideways. This allows precise control without requiring
 * a gyro sensor.
 * 
 * System Integration:
 * - Uses DriveSubsystem for motor control
 * - Reads encoders to track movement distance
 * - Interacts with Constants for configurable parameters
 */
public class Drivetrain_GyroStrafe extends Command {
  
    // Physics constants
    private static final double WHEEL_CIRCUMFERENCE = Constants.Dimensions.WHEEL_DIAMETER * Math.PI;
    private static final double STRAFE_EFFICIENCY_FACTOR = 1.8; // Strafe requires more movement than direct travel
    
    // Command state variables
    private final DriveSubsystem drivetrain = Robot.m_driveSubsystem;
    private double strafePower;     // Power level (0-1.0)
    private double goalDistance;    // Target distance in wheel rotations
    private long startTime;         // For tracking execution time
    private double direction;       // Direction: positive = right, negative = left
    
    // Position tracking 
    private double leftEncoderStart, rightEncoderStart;
    
    /**
     * Creates a new simulated strafe command for tank drive.
     * 
     * @param distance Distance to "strafe" in meters (positive = right, negative = left)
     * @param power Power level for the motion (0.0 to 1.0)
     */
    public Drivetrain_GyroStrafe(double distance, double power) {
        // Calculate direction from the sign of the distance
        direction = Math.signum(distance);
        
        // Store power as a positive value - direction is handled separately
        strafePower = Math.abs(power);
        
        // Convert distance to encoder rotations
        // Apply efficiency factor since tank "strafing" requires more movement
        goalDistance = Math.abs(distance) / WHEEL_CIRCUMFERENCE * STRAFE_EFFICIENCY_FACTOR;
        
        // Register the drivetrain requirement
        addRequirements(drivetrain);
        
        // Log creation information
        System.out.println(">> Creating strafe simulation - Distance: " + 
                         distance + "m, Power: " + power);
    }

    @Override
    public void initialize() {
        System.out.println(">> Starting tank drive strafe simulation");
        System.out.println(">> Direction: " + (direction > 0 ? "RIGHT" : "LEFT"));
        
        // Stop any existing movement
        drivetrain.arcadeDrive(0, 0);
        
        // Record starting encoder positions
        leftEncoderStart = drivetrain.getLeftPosition();
        rightEncoderStart = drivetrain.getRightPosition();
        
        // Record start time for performance metrics
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Simulate strafing by applying opposite power to each track
        // When tracks move in opposite directions, the robot slides sideways
        double leftPower = strafePower * direction;
        double rightPower = -strafePower * direction;
        
        // Apply tank drive with differential powers
        drivetrain.tankDrive(leftPower, rightPower);
        
        // Provide periodic progress updates (every 500ms)
        long elapsedTime = System.currentTimeMillis() - startTime;
        if (elapsedTime > 500 && (elapsedTime % 500 < 20)) {
            // Calculate progress percentage
            double leftTravel = Math.abs(drivetrain.getLeftPosition() - leftEncoderStart);
            double rightTravel = Math.abs(drivetrain.getRightPosition() - rightEncoderStart);
            double avgTravel = (leftTravel + rightTravel) / 2.0;
            double percentComplete = avgTravel / goalDistance * 100;
            
            System.out.printf(">> Strafing: %.1f%% complete\n", percentComplete);
        }
    }

    @Override
    public boolean isFinished() {
        // Calculate distance traveled by each side
        double leftTravel = Math.abs(drivetrain.getLeftPosition() - leftEncoderStart);
        double rightTravel = Math.abs(drivetrain.getRightPosition() - rightEncoderStart);
        
        // Use the average travel distance to determine completion
        double avgTravel = (leftTravel + rightTravel) / 2.0;
        
        // Command is complete when we've traveled the goal distance
        return avgTravel >= goalDistance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motors for safety
        drivetrain.arcadeDrive(0, 0);
        
        if (interrupted) {
            System.out.println(">> Strafe interrupted!");
        } else {
            // Report completion and execution time
            double executionTime = (System.currentTimeMillis() - startTime) / 1000.0;
            
            // Calculate actual distance traveled
            double leftTravel = Math.abs(drivetrain.getLeftPosition() - leftEncoderStart);
            double rightTravel = Math.abs(drivetrain.getRightPosition() - rightEncoderStart);
            double avgTravel = (leftTravel + rightTravel) / 2.0;
            
            System.out.printf(">> Strafe completed in %.1f seconds (traveled %.2f rotations)\n", 
                           executionTime, avgTravel);
        }
    }
}
