// src/main/java/frc/robot/commands/autonomous/BallHuntCommand.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
 * BallHuntCommand - THE ULTIMATE BALL COLLECTION ROUTINE!
 * 
 * This autonomous sequence:
 * 1. Drives out from starting position
 * 2. Does a 16:1-optimized sweep pattern to find balls
 * 3. Uses vision to track and collect balls
 * 4. Returns to scoring zone
 * 5. Scores the balls for MAXIMUM POINTS!
 * 
 * IMPORTANT: Uses encoder-based position tracking since we don't have a gyro
 * 
 * coded by paysean
 */
public class BallHuntCommand extends SequentialCommandGroup {
    
    // ASCII ART BECAUSE IT'S AWESOME!!!
    //    ___     ___     ___     ___  
    //   (o o)   (o o)   (o o)   (o o) 
    //  (  V  ) (  V  ) (  V  ) (  V  )
    //  /--m-\  /--m-\  /--m-\  /--m-\ 
    // BALLS!  BALLS!  BALLS!  BALLS!

    public BallHuntCommand() {
        addCommands(
            // ===== PHASE 1: INITIALIZATION =====
            new InstantCommand(() -> {
                // Reset the encoders for accurate movement
                Robot.m_driveSubsystem.resetEncoders();
                
                // Set vision system to ball tracking mode
                Robot.m_visionSubsystem.setPipeline(0); // Ball detection pipeline
                
                System.out.println("");
                System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                System.out.println(">> BALL HUNT AUTO ROUTINE STARTED!!");
                System.out.println(">> PREPARE FOR BALL DOMINATION!!!!!");
                System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                System.out.println("");
            }),
            
            // ===== PHASE 2: DRIVE TO FIELD CENTER =====
            // Drive forward from starting position - using our 16:1 optimized drive
            new Drivetrain_GyroStraight(1.2, 0.4),
            
            // Small wait to stabilize after initial drive
            new WaitCommand(0.25),
            
            // Turn to face the center of the field (45 degrees)
            new Drivetrain_GyroTurn(45),
            
            // Small wait to stabilize after turn
            new WaitCommand(0.25),
            
            // ===== PHASE 3: LOOK FOR FIRST BALL =====
            // Drive forward a bit more to reach potential ball location
            new Drivetrain_GyroStraight(0.8, 0.3),
            
            // ===== PHASE 4: COLLECT BALL WITH VISION =====
            // Use the vision-based ball tracking to find and collect a ball
            // This command will handle finding, approaching, and collecting
            new BallTrackingCommand(),
            
            // Make sure arm is in safe position with ball
            new InstantCommand(() -> Robot.m_ballArmSubsystem.homeArm()),
            
            // Small wait to ensure arm is stowed
            new WaitCommand(0.5),
            
            // ===== PHASE 5: RETURN TO SCORING ZONE =====
            // Turn back toward starting position (about 170 degrees)
            // Using the 16:1 optimized turn command
            new Drivetrain_GyroTurn(170),
            
            // Small wait to stabilize after big turn
            new WaitCommand(0.3),
            
            // Drive back to scoring position
            new Drivetrain_GyroStraight(1.5, 0.5),
            
            // Final turn to face scoring target
            new Drivetrain_GyroTurn(-15),
            
            // ===== PHASE 6: SCORE THE BALL =====
            // Score the ball we collected!
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Small wait to ensure ball is released
            new WaitCommand(0.3),
            
            // ===== PHASE 7: LOOK FOR MORE BALLS =====
            // Turn back toward field
            new Drivetrain_GyroTurn(180),
            
            // Drive forward to look for more balls
            new Drivetrain_GyroStraight(1.0, 0.4),
            
            // Strafe to expand search area
            new Drivetrain_GyroStrafe(0.6, 0.3),
            
            // Try to collect another ball if there's time
            new BallTrackingCommand().withTimeout(5.0),
            
            // ===== PHASE 8: FINAL CLEANUP =====
            // Return arm to home position
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                System.out.println("");
                System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                System.out.println(">> BALL HUNT AUTO ROUTINE COMPLETE!");
                System.out.println(">> READY FOR TELEOP TAKEOVER!!!!!");
                System.out.println(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                System.out.println("");
            })
        );
    }
}
