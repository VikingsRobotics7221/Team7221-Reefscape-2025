// src/main/java/frc/robot/commands/autonomous/StrategicBallHuntAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.BallControlCommands.ScoreSequence;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;
import frc.robot.commands.autonomous.basic_path_planning.OptimizedTankDriveCommand;

/**
 * StrategicBallHuntAuto - Advanced ball collection autonomous routine
 * 
 * This routine combines precise movement, vision-based tracking, and efficient
 * ball handling to create a high-scoring autonomous sequence. It's designed to:
 * 
 * 1. Navigate quickly to the game field center using optimized drive commands
 * 2. Use vision to locate and collect game pieces efficiently
 * 3. Score collected game pieces for maximum autonomous points
 * 4. Position the robot strategically for teleop takeover
 * 
 * The sequence adapts to the 16:1 gear ratio by using slower, more controlled
 * movements with higher torque. This provides better pushing power and more
 * precise control when tracking balls.
 * 
 * System Connections:
 * - Uses DriveSubsystem for movement control
 * - Uses BallArmSubsystem for game piece manipulation
 * - Uses VisionSubsystem for target identification
 * - Uses path planning commands from the basic_path_planning package
 */
public class StrategicBallHuntAuto extends SequentialCommandGroup {
    
    /**
     * Creates a new autonomous routine for strategic ball collection and scoring.
     * This should be positioned at the starting position near driver station 2.
     */
    public StrategicBallHuntAuto() {
        addCommands(
            // Phase 1: System initialization and diagnostics
            new InstantCommand(() -> {
                // Reset encoders for accurate distance tracking
                Robot.m_driveSubsystem.resetEncoders();
                
                // Configure vision system for ball detection
                Robot.m_visionSubsystem.setPipeline(0); // 0 = Ball tracking pipeline
                
                // Enable turbo mode for quick initial movement
                Robot.m_driveSubsystem.enableTurboMode();
                
                // Log autonomous routine start
                System.out.println("STRATEGIC BALL HUNT AUTO ACTIVATED");
                System.out.println("16:1 DRIVE RATIO OPTIMIZED ROUTINE STARTING");
            }),
            
            // Quick systems verification
            new InstantCommand(() -> {
                // Verify battery is sufficient for auto routine
                double voltage = RobotController.getBatteryVoltage();
                if (voltage < 12.0) {
                    System.out.println("WARNING: Battery voltage low: " + voltage + "V");
                    System.out.println("Performance may be affected");
                }
                
                // Ensure arm is in home position for movement
                Robot.m_ballArmSubsystem.homeArm();
            }),
            
            // Phase 2: Navigate to strategic position
            // Using optimal drive command for 16:1 ratio
            new OptimizedTankDriveCommand(1.8, 0.9, 3.0),
            
            // Turn to face ball collection area
            new Drivetrain_GyroTurn(60),
            
            // Switch to precision mode for fine control during tracking
            new InstantCommand(() -> {
                Robot.m_driveSubsystem.disableDriveModes();
                Robot.m_driveSubsystem.enablePrecisionMode();
                System.out.println("SWITCHED TO PRECISION MODE FOR BALL HUNTING");
            }),
            
            // Brief pause to stabilize before vision tracking
            new WaitCommand(0.2),
            
            // Phase 3: Hunt and collect first ball
            // Short approach movement to get within vision range
            new Drivetrain_GyroStraight(0.5, 0.3),
            
            // Use vision tracking for precise ball acquisition
            new BallTrackingCommand().withTimeout(5.0),
            
            // Secure ball in gripper
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.setGripper(Constants.BallArm.GRIPPER_HOLD_SPEED);
                System.out.println("FIRST BALL ACQUIRED AND SECURED");
            }),
            
            // Return arm to home position with ball
            new InstantCommand(() -> Robot.m_ballArmSubsystem.homeArm()),
            
            // Phase 4: Navigate to scoring position
            // Back up slightly to clear obstructions
            new Drivetrain_GyroStraight(-0.3, 0.4),
            
            // Turn toward scoring zone
            new Drivetrain_GyroTurn(135),
            
            // Switch to turbo mode for fast transit to scoring zone
            new InstantCommand(() -> {
                Robot.m_driveSubsystem.disableDriveModes();
                Robot.m_driveSubsystem.enableTurboMode();
                System.out.println("TURBO MODE ACTIVATED FOR TRANSIT TO SCORING ZONE");
            }),
            
            // Fast drive to scoring position
            new OptimizedTankDriveCommand(1.5, 0.85, 2.5),
            
            // Phase 5: Score the collected ball
            // Switch to precision mode for accurate scoring
            new InstantCommand(() -> {
                Robot.m_driveSubsystem.disableDriveModes();
                Robot.m_driveSubsystem.enablePrecisionMode();
                System.out.println("PRECISION MODE ACTIVATED FOR SCORING");
            }),
            
            // Final alignment adjustment
            new Drivetrain_GyroTurn(-15),
            
            // Score the ball
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Phase 6: Position for teleop
            // Turn to face field center
            new Drivetrain_GyroTurn(180),
            
            // Return to normal drive mode
            new InstantCommand(() -> Robot.m_driveSubsystem.disableDriveModes()),
            
            // Move into strategic position while preparing arm for next ball
            new ParallelCommandGroup(
                // Drive forward while turning slightly to position diagonally
                new OptimizedTankDriveCommand(1.2, 0.6, 2.0),
                
                // While driving, prepare arm for another ball
                new SequentialCommandGroup(
                    new WaitCommand(1.0), // Wait until we're in position
                    new InstantCommand(() -> Robot.m_ballArmSubsystem.pickupPosition())
                )
            ),
            
            // Final turn to optimal defensive position
            new Drivetrain_GyroTurn(45),
            
            // Phase 7: Cleanup and teleop handoff
            new InstantCommand(() -> {
                // Return arm to safe position
                Robot.m_ballArmSubsystem.homeArm();
                
                // Disable special drive modes
                Robot.m_driveSubsystem.disableDriveModes();
                
                // Report completion status
                System.out.println("STRATEGIC BALL HUNT AUTO COMPLETE");
                System.out.println("BALL COLLECTED AND SCORED");
                System.out.println("ROBOT POSITIONED FOR DRIVER CONTROL");
            })
        );
    }
}
