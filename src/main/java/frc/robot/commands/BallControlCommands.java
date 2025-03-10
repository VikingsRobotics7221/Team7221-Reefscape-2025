package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.BallArmSubsystem;

/**
 * Commands for controlling the ball manipulation system.
 * 
 * This file contains all the commands needed to operate the drawer-slide ball arm system.
 * Each command class represents a specific action the robot can perform with the ball system:
 * 
 * - PickupSequence: Collects balls from the floor
 * - ScoreSequence: Scores balls into targets
 * - ManualArmControl: Allows direct control of arm movement
 * - ManualGripperControl: Allows direct control of gripper wheels
 * - ArmToPosition: Moves arm to a specific position
 * 
 * HOW THIS CONNECTS TO OTHER FILES:
 * - Uses BallArmSubsystem to control the physical hardware
 * - Uses Constants for speed values and timing parameters
 * - Commands are bound to controller buttons in Robot.java
 * - Used in autonomous routines for programmed ball handling
 */
public class BallControlCommands {
    
    /**
     * PickupSequence - Automated sequence to collect a ball from the floor
     * 
     * Sequence steps:
     * 1. Move arm to pickup position
     * 2. Wait for arm to reach position
     * 3. Start intake wheels
     * 4. Wait until ball is detected or timeout occurs
     * 5. Secure ball and return arm to home position
     */
    public static class PickupSequence extends SequentialCommandGroup {
        public PickupSequence(BallArmSubsystem ballArm) {
            addCommands(
                // First move arm to pickup position
                new InstantCommand(() -> {
                    System.out.println(">> STARTING BALL PICKUP SEQUENCE!");
                    ballArm.pickupPosition();
                }),
                
                // Wait for arm to get to position
                new WaitCommand(1.0),
                
                // Start the intake
                new InstantCommand(() -> ballArm.setGripper(Constants.BallArm.GRIPPER_INTAKE_SPEED)),
                
                // Wait until we have a ball or timeout after 3 seconds
                new RunCommand(() -> {}).until(() -> ballArm.hasBall()).withTimeout(3.0),
                
                // Once we have the ball, move arm to home position and hold
                new InstantCommand(() -> {
                    ballArm.setGripper(Constants.BallArm.GRIPPER_HOLD_SPEED);
                    ballArm.homeArm();
                    System.out.println(">> PICKUP SEQUENCE COMPLETE!");
                })
            );
        }
    }
    
    /**
     * ScoreSequence - Automated sequence to score a ball into a target
     * 
     * Sequence steps:
     * 1. Move arm to scoring position
     * 2. Wait for arm to reach position
     * 3. Release the ball at high speed
     * 4. Wait for ball to completely exit
     * 5. Return arm to home position
     */
    public static class ScoreSequence extends SequentialCommandGroup {
        public ScoreSequence(BallArmSubsystem ballArm) {
            addCommands(
                // Move arm to scoring position
                new InstantCommand(() -> {
                    System.out.println(">> STARTING SCORING SEQUENCE!");
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
                    System.out.println(">> SCORE SEQUENCE COMPLETE!");
                })
            );
        }
    }
    
    /**
     * ManualArmControl - Command for direct control of arm movement
     * 
     * This command allows direct speed control of the arm extension motor.
     * It's used when the operator wants to manually position the arm.
     * The command runs until interrupted and safely stops the motor when done.
     */
    public static class ManualArmControl extends Command {
        private final BallArmSubsystem m_ballArm;
        private final double m_speed;
        
        /**
         * Creates a manual arm control command
         * 
         * @param ballArm The ball arm subsystem to control
         * @param speed The speed to move the arm (-1.0 to 1.0)
         */
        public ManualArmControl(BallArmSubsystem ballArm, double speed) {
            m_ballArm = ballArm;
            m_speed = speed;
            addRequirements(ballArm);
        }
        
        @Override
        public void initialize() {
            System.out.println(">> Manual arm control - Speed: " + m_speed);
        }
        
        @Override
        public void execute() {
            m_ballArm.moveArm(m_speed);
        }
        
        @Override
        public void end(boolean interrupted) {
            m_ballArm.moveArm(0);
            System.out.println(">> Manual arm control stopped");
        }
        
        @Override
        public boolean isFinished() {
            return false; // Run until interrupted
        }
    }
    
    /**
     * ManualGripperControl - Command for direct control of gripper wheels
     * 
     * This command allows direct speed control of the gripper wheels.
     * It's used when the operator wants to manually control ball intake/release.
     * The command runs until interrupted and safely stops the wheels when done.
     */
    public static class ManualGripperControl extends Command {
        private final BallArmSubsystem m_ballArm;
        private final double m_speed;
        
        /**
         * Creates a manual gripper control command
         * 
         * @param ballArm The ball arm subsystem to control
         * @param speed The speed to run the gripper wheels (-1.0 to 1.0)
         */
        public ManualGripperControl(BallArmSubsystem ballArm, double speed) {
            m_ballArm = ballArm;
            m_speed = speed;
            addRequirements(ballArm);
        }
        
        @Override
        public void initialize() {
            System.out.println(">> Manual gripper control - Speed: " + m_speed);
        }
        
        @Override
        public void execute() {
            m_ballArm.setGripper(m_speed);
        }
        
        @Override
        public void end(boolean interrupted) {
            m_ballArm.setGripper(0);
            System.out.println(">> Manual gripper control stopped");
        }
        
        @Override
        public boolean isFinished() {
            return false; // Run until interrupted
        }
    }
    
    /**
     * ArmToPosition - Command to move arm to a specific position
     * 
     * This command uses the arm's PID controller to move to a precise position.
     * It completes when the arm reaches the target position within tolerance.
     * Used for precise positioning during autonomous or programmed sequences.
     */
    public static class ArmToPosition extends Command {
        private final BallArmSubsystem m_ballArm;
        private final double m_position;
        private static final double POSITION_TOLERANCE = 0.2;  // acceptable error in rotations
        
        /**
         * Creates a command to move the arm to a specific position
         * 
         * @param ballArm The ball arm subsystem to control
         * @param position The target position in encoder rotations
         */
        public ArmToPosition(BallArmSubsystem ballArm, double position) {
            m_ballArm = ballArm;
            m_position = position;
            addRequirements(ballArm);
        }
        
        @Override
        public void initialize() {
            System.out.println(">> Moving arm to position: " + m_position);
        }
        
        @Override
        public void execute() {
            m_ballArm.setArmPosition(m_position);
        }
        
        @Override
        public void end(boolean interrupted) {
            // Keep arm at current position
            if (interrupted) {
                System.out.println(">> Arm position move interrupted!");
            } else {
                System.out.println(">> Arm at target position!");
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
