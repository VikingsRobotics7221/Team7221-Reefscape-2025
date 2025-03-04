// src/main/java/frc/robot/commands/autonomous/BallCollectionAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Robot;
import frc.robot.commands.BallTrackingCommand; // FIXED: Changed from BallTargetingCommand
import frc.robot.commands.BallControlCommands;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

/**
 * BallCollectionAuto - AUTONOMOUS BALL HOARDING!
 * 
 * This auto routine:
 * 1. Drives out from starting position 
 * 2. Uses vision to find and collect a ball
 * 3. Returns to scoring position
 * 4. Scores the ball
 * 5. Repeats for a second ball if time allows
 * 
 * Total autonomous domination! >>
 * 
 * coded by paysean
 */
public class BallCollectionAuto extends SequentialCommandGroup {
    
    public BallCollectionAuto() {
        addCommands(
            // ===== PHASE 1: SETUP =====
            new InstantCommand(() -> {
                // Zero gyro for proper navigation
                Robot.m_driveSubsystem.zeroGyro();
                
                // Set vision to ball detection pipeline
                Robot.m_visionSubsystem.setPipeline(0);
                
                System.out.println(">> STARTING BALL COLLECTION AUTO");
            }),
            
            // ===== PHASE 2: MOVE TO FIELD CENTER =====
            // Drive forward to reach the field
            new Drivetrain_GyroStraight(1.0, 0.4),
            
            // Turn slightly to face the field center
            new Drivetrain_GyroTurn(15),
            
            // ===== PHASE 3: COLLECT FIRST BALL =====
            // Run vision-based ball targeting sequence
            new BallTrackingCommand(), // FIXED: Changed from BallTargetingCommand
            
            // Make sure arm is safely stowed after collection
            new InstantCommand(() -> Robot.m_ballArmSubsystem.homeArm()),
            
            // Small wait to ensure arm is stowed
            new WaitCommand(0.5),
            
            // ===== PHASE 4: RETURN TO SCORING POSITION =====
            // Turn back toward starting position/scoring area
            new Drivetrain_GyroTurn(165),
            
            // Drive back to scoring position
            new Drivetrain_GyroStraight(1.2, 0.5),
            
            // Final alignment turn to face target
            new Drivetrain_GyroTurn(-10),
            
            // ===== PHASE 5: SCORE FIRST BALL =====
            // Run scoring sequence
            new BallControlCommands.ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Small wait to ensure scoring is complete
            new WaitCommand(0.5),
            
            // ===== PHASE 6: GO FOR SECOND BALL IF TIME ALLOWS =====
            // Turn back toward field
            new Drivetrain_GyroTurn(-155),
            
            // Drive toward field center again
            new Drivetrain_GyroStraight(1.3, 0.6),
            
            // Collect second ball
            new BallTrackingCommand(), // FIXED: Changed from BallTargetingCommand
            
            // ===== PHASE 7: WRAP UP =====
            // Make sure everything is in safe position at end
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                System.out.println(">> BALL COLLECTION AUTO COMPLETE!");
            })
        );
    }
}
