// src/main/java/frc/robot/commands/autonomous/DefensiveHoardingAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.BallControlCommands.PickupSequence;
import frc.robot.commands.BallControlCommands.ScoreSequence;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStrafe;

/**
 * DefensiveHoardingAuto - Strategic Ball Control Autonomous Routine
 * 
 * This autonomous command sequence implements a defensive strategy for the Reefscape game.
 * The robot will:
 * 1. Rush to center field to collect loose balls
 * 2. Block key scoring paths while holding collected balls
 * 3. Reposition to collect additional balls
 * 4. Return to scoring position for endgame points
 * 
 * System Integration:
 * - Uses the DriveSubsystem for all movement commands
 * - Uses BallArmSubsystem for ball collection and scoring
 * - Uses VisionSubsystem for target detection
 * - Designed for the 16:1 gearing on our drivetrain
 * 
 * This routine is optimized for the starting position near driver station 2.
 */
public class DefensiveHoardingAuto extends SequentialCommandGroup {
    
    /**
     * Creates a new DefensiveHoardingAuto command sequence
     */
    public DefensiveHoardingAuto() {
        addCommands(
            // ---------- PHASE 1: INITIALIZATION ----------
            new InstantCommand(() -> {
                // Reset sensors and prepare systems
                Robot.m_driveSubsystem.resetEncoders();
                Robot.m_driveSubsystem.enableTurboMode();
                Robot.m_visionSubsystem.setPipeline(0); // Ball detection mode
                
                System.out.println("DEFENSIVE HOARDING AUTO INITIALIZED");
                System.out.println("EXECUTING BALL CONTROL STRATEGY");
            }),
            
            // ---------- PHASE 2: MOVE TO CENTER FIELD ----------
            // Fast drive to center field using 16:1 drivetrain
            new Drivetrain_GyroStraight(2.0, 0.9),
            
            // Quick scan turn to locate balls
            new Drivetrain_GyroTurn(30),
            
            // ---------- PHASE 3: FIRST BALL COLLECTION ----------
            // Use vision tracking to find and collect first ball
            new BallTrackingCommand().withTimeout(5.0),
            
            // Secure the ball and return arm to safe position
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                Robot.m_ballArmSubsystem.setGripper(Constants.BallArm.GRIPPER_HOLD_SPEED);
                System.out.println("FIRST BALL SECURED - PROCEEDING TO DEFENSIVE POSITION");
            }),
            
            // ---------- PHASE 4: DEFENSIVE POSITIONING ----------
            // Turn to block a key scoring lane
            new Drivetrain_GyroTurn(45),
            
            // Strafe to optimal blocking position
            new Drivetrain_GyroStrafe(0.8, 0.4),
            
            // Maintain blocking position for 3 seconds
            new RunCommand(() -> {
                // Apply slight forward pressure to maintain position
                Robot.m_driveSubsystem.arcadeDrive(0.1, 0);
                System.out.println("DEFENSIVE BLOCKING ACTIVE");
            }).withTimeout(3.0),
            
            // Disable turbo mode for more controlled movement
            new InstantCommand(() -> Robot.m_driveSubsystem.disableDriveModes()),
            
            // ---------- PHASE 5: SECOND BALL COLLECTION ----------
            // Turn to scan for more balls
            new Drivetrain_GyroTurn(-90),
            
            // Drive forward to look for another ball
            new Drivetrain_GyroStraight(1.2, 0.5),
            
            // Score first ball to make room for second collection
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Collect another ball
            new BallTrackingCommand().withTimeout(4.0),
            
            // ---------- PHASE 6: RETURN TO SCORING POSITION ----------
            // Turn back toward scoring zone
            new Drivetrain_GyroTurn(135),
            
            // Drive to scoring position
            new Drivetrain_GyroStraight(1.8, 0.6),
            
            // Final alignment turn
            new Drivetrain_GyroTurn(-15),
            
            // ---------- PHASE 7: FINAL SCORING ----------
            // Score the second ball
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Return to safe position
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                Robot.m_driveSubsystem.arcadeDrive(0, 0);
                System.out.println("DEFENSIVE AUTO COMPLETE - READY FOR TELEOP");
            })
        );
    }
}
