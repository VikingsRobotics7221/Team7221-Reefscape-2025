// src/main/java/frc/robot/commands/autonomous/ReefscapeAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Robot;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.BallControlCommands.ScoreSequence;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

/**
 * ReefscapeAuto - Team 7221's GYRO-FREE AUTONOMOUS BALL COLLECTION!
 * 
 * This autonomous routine has been updated to work without a gyro by using
 * time-based turns and encoder-driven distance measurement.
 * 
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
                // Reset encoders for navigation
                Robot.m_driveSubsystem.resetEncoders();
                
                // Set vision to ball detection mode
                Robot.m_visionSubsystem.setPipeline(0);
                
                System.out.println("");
                System.out.println("======================================");
                System.out.println(">> STARTING REEFSCAPE AUTO ROUTINE!  ");
                System.out.println(">> GYRO-FREE VERSION - LET'S DO THIS!");
                System.out.println("======================================");
                System.out.println("");
            }),
            
            // Drive forward into field
            new Drivetrain_GyroStraight(1.0, 0.4),
            
            // Turn to scan the field (now using timer-based turn)
            new Drivetrain_GyroTurn(45),
            
            // Brief pause to stabilize
            new WaitCommand(0.2),
            
            // Find and collect first ball
            new BallTrackingCommand(),
            
            // Wait for arm to be safely stowed
            new WaitCommand(0.5),
            
            // Turn back toward scoring zone (time-based 180 degree turn)
            new Drivetrain_GyroTurn(180),
            
            // Brief pause to stabilize
            new WaitCommand(0.2),
            
            // Drive back to scoring position
            new Drivetrain_GyroStraight(1.2, 0.5),
            
            // Final alignment turn to face target
            new Drivetrain_GyroTurn(-10),
            
            // Brief pause before scoring
            new WaitCommand(0.3),
            
            // Score the ball
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Cleanup - make sure arm is stowed
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                System.out.println("");
                System.out.println("======================================");
                System.out.println(">> AUTO ROUTINE COMPLETE - SUCCESS!  ");
                System.out.println(">> SWITCHING TO TELEOP CONTROL       ");
                System.out.println("======================================");
                System.out.println("");
            })
        );
    }
}
