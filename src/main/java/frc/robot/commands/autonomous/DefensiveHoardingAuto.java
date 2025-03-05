// src/main/java/frc/robot/commands/autonomous/DefensiveHoardingAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Robot;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.BallControlCommands.PickupSequence;
import frc.robot.commands.BallControlCommands.ScoreSequence;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStrafe;

/**
 * DefensiveHoardingAuto - THE ULTIMATE DEFENSIVE STRATEGY!
 * 
 * This autonomous sequence is built for our 16:1 drive ratio and
 * focuses on ball hoarding while blocking opponent scoring paths!
 * 
 * Strategy:
 * 1. Rush to center field to collect loose balls
 * 2. Hoard balls to prevent opponent scoring
 * 3. Block key scoring lanes with our chassis
 * 4. Score our hoarded balls in the final seconds
 * 
 * coded by paysean
 */
public class DefensiveHoardingAuto extends SequentialCommandGroup {
    
    /*
     *   _______  _______  _______  _______  _______  _______ 
     *  |       ||       ||       ||       ||       ||       |
     *  |_     _||_     _||_     _||_     _||_     _||_     _|
     *    |   |    |   |    |   |    |   |    |   |    |   |  
     *    |___|    |___|    |___|    |___|    |___|    |___|  
     *   VIKING    VIKING    VIKING   VIKING   VIKING   VIKING
     *    POWER     POWER     POWER    POWER    POWER    POWER
     */
    
    public DefensiveHoardingAuto() {
        addCommands(
            // ===== PHASE 1: AGGRESSIVE START =====
            new InstantCommand(() -> {
                // Reset encoders for accurate positioning
                Robot.m_driveSubsystem.resetEncoders();
                
                // Enable turbo mode for maximum speed to center field!
                Robot.m_driveSubsystem.enableTurboMode();
                
                // Set vision to ball detection
                Robot.m_visionSubsystem.setPipeline(0);
                
                System.out.println("");
                System.out.println("»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»");
                System.out.println("» DEFENSIVE HOARDING AUTO ACTIVATED «");
                System.out.println("» PREPARE FOR MAXIMUM BALL CONTROL! «");
                System.out.println("»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»");
                System.out.println("");
            }),
            
            // ===== PHASE 2: RUSH TO CENTER FIELD =====
            // Fast drive to center field using our 16:1 drivetrain at max power
            new Drivetrain_GyroStraight(2.0, 0.9),
            
            // Quick scan turn to locate balls
            new Drivetrain_GyroTurn(30),
            
            // ===== PHASE 3: FIRST BALL COLLECTION =====
            // Use vision tracking to find and collect first ball
            new BallTrackingCommand().withTimeout(5.0),
            
            // Home the arm to secure the ball
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                Robot.m_ballArmSubsystem.setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
                System.out.println(">> FIRST BALL SECURED - HOARDING ENGAGED!");
            }),
            
            // ===== PHASE 4: DEFENSIVE POSITIONING =====
            // Turn to block a key lane
            new Drivetrain_GyroTurn(45),
            
            // Strafe to optimal blocking position
            new Drivetrain_GyroStrafe(0.8, 0.4),
            
            // Wait in blocking position (3 seconds)
            new RunCommand(() -> {
                // Apply slight forward pressure to maintain position
                Robot.m_driveSubsystem.arcadeDrive(0.1, 0);
                System.out.println(">> DEFENSIVE BLOCKING ACTIVE!");
            }).withTimeout(3.0),
            
            // Disable turbo mode for more controlled movement
            new InstantCommand(() -> Robot.m_driveSubsystem.disableDriveModes()),
            
            // ===== PHASE 5: SECOND BALL COLLECTION =====
            // Turn to scan for more balls
            new Drivetrain_GyroTurn(-90),
            
            // Drive forward to look for another ball
            new Drivetrain_GyroStraight(1.2, 0.5),
            
            // We can't pick up another ball with our first one, so let's score it
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Now collect another ball
            new BallTrackingCommand().withTimeout(4.0),
            
            // ===== PHASE 6: RETURN TO SCORING POSITION =====
            // Turn back toward scoring zone
            new Drivetrain_GyroTurn(135),
            
            // Drive to scoring position
            new Drivetrain_GyroStraight(1.8, 0.6),
            
            // Final alignment turn
            new Drivetrain_GyroTurn(-15),
            
            // ===== PHASE 7: FINAL SCORING =====
            // Score the second ball
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Return to safe position
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                Robot.m_driveSubsystem.arcadeDrive(0, 0);
                System.out.println("");
                System.out.println("»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»");
                System.out.println("» DEFENSIVE AUTO COMPLETE - SUCCESS! «");
                System.out.println("» BALLS HOARDED AND SCORED!         «");
                System.out.println("»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»»");
                System.out.println("");
            })
        );
    }
}
