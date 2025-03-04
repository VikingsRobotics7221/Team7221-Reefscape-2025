// src/main/java/frc/robot/commands/autonomous/ReefscapeAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Robot;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.BallControlCommands.ScoreSequence;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

/**
 * ReefscapeAuto - Team 7221's automated ball collection strategy!
 * 
 * This autonomous routine:
 * 1. Moves from starting position
 * 2. Searches for and collects balls
 * 3. Returns to scoring zone
 * 4. Scores the collected balls
 * 
 * coded by paysean
 */
public class ReefscapeAuto extends SequentialCommandGroup {
    
    public ReefscapeAuto() {
        addCommands(
            // Initialize systems
            new InstantCommand(() -> {
                // Zero gyro for navigation
                Robot.m_driveSubsystem.zeroGyro();
                
                // Set vision to ball detection mode
                Robot.m_visionSubsystem.setPipeline(0);
                
                System.out.println("🚀 STARTING REEFSCAPE AUTO ROUTINE");
            }),
            
            // Drive forward into field
            new Drivetrain_GyroStraight(1.0, 0.4),
            
            // Turn to scan the field
            new Drivetrain_GyroTurn(45),
            
            // Find and collect first ball
            new BallTrackingCommand(),
            
            // Wait for arm to be safely stowed
            new WaitCommand(0.5),
            
            // Turn back toward scoring zone
            new Drivetrain_GyroTurn(180),
            
            // Drive back to scoring position
            new Drivetrain_GyroStraight(1.2, 0.5),
            
            // Final alignment turn
            new Drivetrain_GyroTurn(-10),
            
            // Score the ball
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Cleanup - make sure arm is stowed
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                System.out.println("✅ AUTO ROUTINE COMPLETE");
            })
        );
    }
}
