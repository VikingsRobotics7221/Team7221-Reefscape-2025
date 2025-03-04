// src/main/java/frc/robot/commands/BallControlCommands.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.BallArmSubsystem;

/**
 * Commands for controlling the ball system
 * 
 * This is like the game plan for our ball handling system.
 * Each command here is a specific play we can run during the match!
 * 
 * coded by paysean
 */
public class BallControlCommands {
    
    // PICKUP SEQUENCE - Gets a ball from the floor
    public static class PickupSequence extends SequentialCommandGroup {
        public PickupSequence(BallArmSubsystem ballArm) {
            addCommands(
                // First move arm to pickup position
                new InstantCommand(() -> {
                    System.out.println("🏀 STARTING BALL PICKUP SEQUENCE!");
                    ballArm.pickupPosition();
                }),
                
                // Wait for arm to get to position
                new WaitCommand(1.0),
                
                // Start the intake
                new InstantCommand(() -> ballArm.setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED)),
                
                // Wait until we have a ball or timeout after 3 seconds
                new RunCommand(() -> {}).until(() -> ballArm.hasBall()).withTimeout(3.0),
                
                // Once we have the ball, move arm to home position and hold
                new InstantCommand(() -> {
                    ballArm.setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
                    ballArm.homeArm();
                    System.out.println("✅ PICKUP SEQUENCE COMPLETE!");
                })
            );
        }
    }
    
    // SCORE SEQUENCE - Launches a ball at the target
    public static class ScoreSequence extends SequentialCommandGroup {
        public ScoreSequence(BallArmSubsystem ballArm) {
            addCommands(
                // Move arm to scoring position
                new InstantCommand(() -> {
                    System.out.println("🎯 STARTING SCORING SEQUENCE!");
                    ballArm.scorePosition();
                }),
                
                // Wait for arm to get to position
                new WaitCommand(1.0),
                
                // Release the ball!
                new InstantCommand(() -> ballArm.releaseBall()),
                
                // Wait for ball to completely leave
                new WaitCommand(0.5),
                
                // Return to home position
                new InstantCommand(() -> {
                    ballArm.setGripper(0);
                    ballArm.homeArm();
                    System.out.println("🎊 SCORE SEQUENCE COMPLETE!");
                })
            );
        }
    }
    
    // MANUAL ARM CONTROL - Basic class for manual arm control
    public static class ManualArmControl extends Command {
        private final BallArmSubsystem m_ballArm;
        private final double m_speed;
        
        public ManualArmControl(BallArmSubsystem ballArm, double speed) {
            m_ballArm = ballArm;
            m_speed = speed;
            addRequirements(ballArm);
        }
        
        @Override
        public void initialize() {
            System.out.println("🕹️ Manual arm control - Speed: " + m_speed);
        }
        
        @Override
        public void execute() {
            m_ballArm.moveArm(m_speed);
        }
        
        @Override
        public void end(boolean interrupted) {
            m_ballArm.moveArm(0);
            System.out.println("🛑 Manual arm control stopped");
        }
        
        @Override
        public boolean isFinished() {
            return false; // Run until interrupted
        }
    }
    
    // MANUAL GRIPPER CONTROL - Basic class for manual gripper control
    public static class ManualGripperControl extends Command {
        private final BallArmSubsystem m_ballArm;
        private final double m_speed;
        
        public ManualGripperControl(BallArmSubsystem ballArm, double speed) {
            m_ballArm = ballArm;
            m_speed = speed;
            addRequirements(ballArm);
        }
        
        @Override
        public void initialize() {
            System.out.println("🔄 Manual gripper control - Speed: " + m_speed);
        }
        
        @Override
        public void execute() {
            m_ballArm.setGripper(m_speed);
        }
        
        @Override
        public void end(boolean interrupted) {
            m_ballArm.setGripper(0);
            System.out.println("🛑 Manual gripper control stopped");
        }
        
        @Override
        public boolean isFinished() {
            return false; // Run until interrupted
        }
    }
    
    // GO TO POSITION - Command to move arm to a specific position
    public static class ArmToPosition extends Command {
        private final BallArmSubsystem m_ballArm;
        private final double m_position;
        private static final double POSITION_TOLERANCE = 0.2;  // acceptable error in rotations
        
        public ArmToPosition(BallArmSubsystem ballArm, double position) {
            m_ballArm = ballArm;
            m_position = position;
            addRequirements(ballArm);
        }
        
        @Override
        public void initialize() {
            System.out.println("🎯 Moving arm to position: " + m_position);
        }
        
        @Override
        public void execute() {
            m_ballArm.setArmPosition(m_position);
        }
        
        @Override
        public void end(boolean interrupted) {
            // Keep arm at current position
            if (interrupted) {
                System.out.println("⚠️ Arm position move interrupted!");
            } else {
                System.out.println("✅ Arm at target position!");
            }
        }
        
        @Override
        public boolean isFinished() {
            // Check if we're at the target position
            double error = Math.abs(m_ballArm.getArmPosition() - m_position);
            return error < POSITION_TOLERANCE;
        }
    }
}
