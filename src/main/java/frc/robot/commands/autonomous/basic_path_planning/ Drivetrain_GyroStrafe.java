// Author: Team 7221
// Last Updated: March 2025

package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

import frc.robot.subsystems.DriveSubsystem;

/** 
 * Drivetrain_GyroStrafe - NO-GYRO TANK DRIVE STRAFE SIMULATION!
 * 
 * This command simulates strafing motion on a tank drive by applying differential
 * power to create a "crab-like" movement. Since we removed the gyro, we now use
 * encoder feedback to determine when we've moved enough distance.
 * 
 * It's honestly amazing how well this works without a gyro!!! The differential
 * wheel speeds create a sort of "sliding" motion that *feels* like strafing.
 * 
 * FUN FACT: Real tanks can't actually strafe - this is just a clever trick!
 * 
 * coded by paysean
 */
public class Drivetrain_GyroStrafe extends Command {
  
    // ======= CONSTANTS - CRITICAL VALUES FOR PERFECT STRAFING =======
    private static final double CIRCUMFRENCE = Constants.WHEEL_DIAMETER * Math.PI;
    private static final double STRAFE_SPEED_MULTIPLIER = 1.8; // Since "strafing" is less efficient than driving
    
    // ======= INSTANCE VARIABLES - TRACKING OUR MOVEMENT =======
    private final DriveSubsystem drivetrain = Robot.m_driveSubsystem;
    private double strafePower; // Power level for the strafe motion
    private double goalDistance; // How far to "strafe" in wheel rotations
    private long startTime; // For tracking command duration
    private double leftEncoderStart, rightEncoderStart; // Store starting positions
    
    // Direction tracking (positive = right, negative = left)
    private double direction;
    
    /**
     * Creates a new "Strafe" command for tank drive (without gyro)
     * 
     * @param distance Distance to "strafe" in meters
     * @param power Power level for the motion (0.0 to 1.0)
     */
    public Drivetrain_GyroStrafe(double distance, double power) {
        // Set direction based on sign of distance
        direction = Math.signum(distance);
        
        // Store the power level (always positive - direction handled separately)
        strafePower = Math.abs(power);
        
        // Convert distance to encoder rotations, but multiply by our "strafe factor"
        // because tank drive strafing is less efficient than regular driving
        goalDistance = Math.abs(distance) / CIRCUMFRENCE * STRAFE_SPEED_MULTIPLIER;
        
        // This command requires the drivetrain subsystem
        addRequirements(drivetrain);
        
        System.out.println(">> CREATING STRAFE SIMULATION - DISTANCE: " + 
                          distance + "m, POWER: " + power);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println(">> STARTING TANK DRIVE STRAFE SIMULATION!");
        System.out.println(">> DIRECTION: " + (direction > 0 ? "RIGHT" : "LEFT"));
        
        // Stop any existing movement
        drivetrain.arcadeDrive(0, 0);
        
        // Record starting encoder positions
        leftEncoderStart = drivetrain.getLeftFrontPosition();
        rightEncoderStart = drivetrain.getRightFrontPosition();
        
        // Record start time for diagnostics
        startTime = System.currentTimeMillis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // OMG THIS IS SO COOL - We simulate strafing by applying opposing power to the tracks
        // When one side goes forward and the other backward, we get a "sliding" motion!
        
        // Direction determines which side goes which way
        double leftPower = strafePower * direction;
        double rightPower = -strafePower * direction;
        
        // Apply tank drive with our differential powers
        drivetrain.tankDrive(leftPower, rightPower);
        
        // Debug info
        if (System.currentTimeMillis() - startTime > 500 && 
           (System.currentTimeMillis() - startTime) % 500 < 20) {
            
            // Calculate how far we've gone
            double leftTravel = Math.abs(drivetrain.getLeftFrontPosition() - leftEncoderStart);
            double rightTravel = Math.abs(drivetrain.getRightFrontPosition() - rightEncoderStart);
            double avgTravel = (leftTravel + rightTravel) / 2.0;
            
            // Report progress
            System.out.println(">> STRAFING: " + 
                              (avgTravel / goalDistance * 100) + 
                              "% complete");
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Calculate distance traveled by each side
        double leftTravel = Math.abs(drivetrain.getLeftFrontPosition() - leftEncoderStart);
        double rightTravel = Math.abs(drivetrain.getRightFrontPosition() - rightEncoderStart);
        
        // Use the average travel distance to determine completion
        double avgTravel = (leftTravel + rightTravel) / 2.0;
        
        // We're done when the average travel exceeds our goal
        if (avgTravel >= goalDistance) {
            System.out.println(">> STRAFE COMPLETE! TRAVELED: " + avgTravel + " rotations");
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // SAFETY FIRST - Stop all motors
        drivetrain.arcadeDrive(0, 0);
        
        if (interrupted) {
            System.out.println(">> STRAFE INTERRUPTED!");
        } else {
            // Calculate execution time
            double executionTime = (System.currentTimeMillis() - startTime) / 1000.0;
            System.out.println(">> STRAFE COMPLETED IN " + executionTime + " SECONDS!");
        }
    }
}
