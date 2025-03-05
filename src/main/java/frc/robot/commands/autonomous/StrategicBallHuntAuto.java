// src/main/java/frc/robot/commands/autonomous/StrategicBallHuntAuto.java
package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Robot;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.BallControlCommands;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStrafe;
import frc.robot.commands.autonomous.basic_path_planning.OptimizedTankDriveCommand;

/**
 * StrategicBallHuntAuto - THE ULTIMATE BALL HUNTING SEQUENCE!
 * 
 * This autonomous routine combines our 16:1 ratio optimized drive commands
 * with our vision system to create the PERFECT ball collection strategy.
 * 
 * Strategy Overview:
 * 1. Fast initial drive to center field using our 16:1 power
 * 2. Quick scan rotation to locate balls
 * 3. Vision-based tracking and collection
 * 4. Strategic positioning near scoring zones
 * 5. Ball scoring
 * 6. Defense positioning
 * 
 * THIS ROUTINE WILL CRUSH THE COMPETITION!!!
 * 
 * coded by paysean - March 2025
 */
public class StrategicBallHuntAuto extends SequentialCommandGroup {
    
    // ASCII ART FOR THE WIN!!!
    //    _____                 _____  
    //   / ____|               |  __ \ 
    //  | |  __ ___  ____ _  __| |__) |
    //  | | |_ / _ \|  _ | |/ /|  _  / 
    //  | |__| | (_) | | | | |_| | \ \ 
    //   \_____\___/|_| |_|\__|_|  \_\
    //                                 
    
    /**
     * Creates our ultimate autonomous routine!
     * MUST be aligned with the starting position by driver station 2
     */
    public StrategicBallHuntAuto() {
        addCommands(
            // ===== PHASE 1: INITIALIZE AND DIAGNOSTICS =====
            new InstantCommand(() -> {
                // Reset encoders for accurate navigation
                Robot.m_driveSubsystem.resetEncoders();
                
                // Set vision to ball detection mode (pipeline 0)
                Robot.m_visionSubsystem.setPipeline(0);
                
                // Enable turbo mode for fast initial movement
                Robot.m_driveSubsystem.enableTurboMode();
                
                System.out.println("");
                System.out.println("◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤");
                System.out.println(">> STRATEGIC BALL HUNT AUTO ACTIVATED!!");
                System.out.println(">> 16:1 DRIVE RATIO ENGAGED!!");
                System.out.println(">> MAXIMUM TORQUE = MAXIMUM DOMINATION!!");
                System.out.println("◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣");
                System.out.println("");
            }),
            
            // Quick systems check to verify we're ready
            new InstantCommand(() -> {
                double voltage = RobotController.getBatteryVoltage();
                if (voltage < 12.0) {
                    System.out.println("!! WARNING: Battery voltage low: " + voltage + "V !!");
                    System.out.println("!! Performance may be affected !!");
                }
                
                // Make sure arm is in home position before moving
                Robot.m_ballArmSubsystem.homeArm();
            }),
            
            // ===== PHASE 2: RUSH TO STRATEGIC POSITION =====
            // Using our optimized drive command specifically tuned for 16:1 ratio!
            new OptimizedTankDriveCommand(1.8, 0.9, 3.0),
            
            // Fast 60-degree turn to face ball collection area
            new Drivetrain_GyroTurn(60),
            
            // Switch to precision mode for better control during ball hunting
            new InstantCommand(() -> {
                Robot.m_driveSubsystem.disableDriveModes();
                Robot.m_driveSubsystem.enablePrecisionMode();
                System.out.println(">> SWITCHED TO PRECISION MODE FOR HUNTING <<");
            }),
            
            // Brief pause to let things settle
            new WaitCommand(0.2),
            
            // ===== PHASE 3: HUNT FIRST BALL =====
            // Drive forward slowly to approach ball area
            new Drivetrain_GyroStraight(0.5, 0.3),
            
            // Use vision to find and collect ball
            new BallTrackingCommand().withTimeout(5.0), 
            
            // Make sure arm is securely holding the ball
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
                System.out.println(">> FIRST BALL ACQUIRED! SECURING... <<");
            }),
            
            // Return to home position with ball
            new InstantCommand(() -> Robot.m_ballArmSubsystem.homeArm()),
            
            // ===== PHASE 4: STRATEGIC REPOSITIONING =====
            // Back up slightly to clear space
            new Drivetrain_GyroStraight(-0.3, 0.4),
            
            // Turn toward scoring zone
            new Drivetrain_GyroTurn(135),
            
            // Switch back to turbo mode for fast transit
            new InstantCommand(() -> {
                Robot.m_driveSubsystem.disableDriveModes();
                Robot.m_driveSubsystem.enableTurboMode();
                System.out.println(">> TURBO MODE ACTIVATED FOR STRATEGIC REPOSITIONING <<");
            }),
            
            // Drive quickly to scoring zone
            new OptimizedTankDriveCommand(1.5, 0.85, 2.5),
            
            // ===== PHASE 5: SCORING SEQUENCE =====
            // Switch back to precision mode for scoring
            new InstantCommand(() -> {
                Robot.m_driveSubsystem.disableDriveModes();
                Robot.m_driveSubsystem.enablePrecisionMode();
                System.out.println(">> PRECISION MODE ACTIVATED FOR SCORING <<");
            }),
            
            // Final alignment turn
            new Drivetrain_GyroTurn(-15),
            
            // Score the ball!
            new BallControlCommands.ScoreSequence(Robot.m_ballArmSubsystem),
            
            // ===== PHASE 6: DEFENSE POSITIONING =====
            // Turn to face field center
            new Drivetrain_GyroTurn(180),
            
            // Back to normal drive mode
            new InstantCommand(() -> Robot.m_driveSubsystem.disableDriveModes()),
            
            // Position to block opponent scoring paths
            new ParallelCommandGroup(
                // Drive forward while turning slightly to position diagonally
                new OptimizedTankDriveCommand(1.2, 0.6, 2.0),
                
                // While driving, prepare arm for another ball
                new SequentialCommandGroup(
                    new WaitCommand(1.0), // Wait until we're in position
                    new InstantCommand(() -> Robot.m_ballArmSubsystem.pickupPosition())
                )
            ),
            
            // ===== PHASE 7: ENDGAME PREPARATION =====
            // Final positioning for teleop takeover
            new Drivetrain_GyroTurn(45),
            
            // Return everything to safe state
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                Robot.m_driveSubsystem.disableDriveModes();
                
                System.out.println("");
                System.out.println("◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤◢◤");
                System.out.println(">> STRATEGIC AUTO COMPLETE - MISSION SUCCESS!!!");
                System.out.println(">> BALL COLLECTED, SCORED, AND POSITIONED FOR DEFENSE!");
                System.out.println(">> READY FOR DRIVER TAKEOVER!");
                System.out.println("◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣◥◣");
                System.out.println("");
            })
        );
    }
}
