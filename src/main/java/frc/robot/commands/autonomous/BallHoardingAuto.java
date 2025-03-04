package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// FIXED: Removed unused import - import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.BallControlCommands;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

/**
 * BALL HOARDING AUTO - The ultimate ball collection strategy!
 * 
 * This auto routine is our secret weapon! We drive around the field,
 * collect as many balls as possible, then bring them back to our zone.
 * It's basically like stealing all the candy at Halloween! 😈
 * 
 * coded by paysean
 */
public class BallHoardingAuto extends SequentialCommandGroup {
    
    public BallHoardingAuto() {
        addCommands(
            // ===== PHASE 1: FIRST BALL =====
            new InstantCommand(() -> System.out.println("🤖 BEGINNING OPERATION BALL HOARD")),
            
            // Drive to first ball
            new Drivetrain_GyroStraight(1.2, 0.4),
            
            // Grab ball
            new BallControlCommands.PickupSequence(Robot.m_ballArmSubsystem),
            
            // ===== PHASE 2: SECOND BALL =====
            // Turn toward second ball
            new Drivetrain_GyroTurn(45),
            
            // Drive to second ball
            new Drivetrain_GyroStraight(0.8, 0.3),
            
            // Home arm to safe position before continuing
            new InstantCommand(() -> Robot.m_ballArmSubsystem.homeArm()),
            
            // Wait for arm to complete movement
            new WaitCommand(0.5),
            
            // Turn more to approach the ball from right angle
            new Drivetrain_GyroTurn(25),
            
            // Keep driving to align with second ball
            new Drivetrain_GyroStraight(0.4, 0.3),
            
            // Score the first ball to make room for second ball
            new BallControlCommands.ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Immediately go to pickup position for next ball
            new InstantCommand(() -> Robot.m_ballArmSubsystem.pickupPosition()),
            
            // Drive forward to reach the ball
            new Drivetrain_GyroStraight(0.5, 0.25),
            
            // Grab second ball
            new InstantCommand(() -> Robot.m_ballArmSubsystem.setGripper(0.8))
                .andThen(new WaitCommand(1.0))
                .andThen(new InstantCommand(() -> Robot.m_ballArmSubsystem.homeArm())),
                
            // ===== PHASE 3: RETURN TO SAFETY =====
            // Turn back toward starting position
            new Drivetrain_GyroTurn(180),
            
            // Drive back to starting position
            new Drivetrain_GyroStraight(1.5, 0.6),
            
            // Final turn to face the target
            new Drivetrain_GyroTurn(-10),
            
            // Score the second ball
            new BallControlCommands.ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Safety - return arm to home position
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                System.out.println("✅ BALL HOARDING MISSION COMPLETE");
            })
        );
    }
}
