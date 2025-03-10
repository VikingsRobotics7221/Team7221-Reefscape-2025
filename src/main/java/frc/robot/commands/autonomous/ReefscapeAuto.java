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
 * ReefscapeAuto - Main autonomous routine for the 2025 Reefscape competition
 * 
 * This autonomous sequence performs the following operations:
 * 1. Drive forward from starting position to reach field center
 * 2. Scan the field to locate game pieces
 * 3. Track and collect a game piece using vision
 * 4. Return to scoring zone
 * 5. Score the collected game piece
 * 
 * Key connections with other systems:
 * - Uses DriveSubsystem for movement (controlled via GyroStraight and GyroTurn commands)
 * - Uses VisionSubsystem for ball detection (switched to pipeline 0)
 * - Uses BallArmSubsystem for game piece collection and scoring
 * 
 * This routine is designed to work without a physical gyro by using encoder-based
 * turning and position tracking.
 */
public class ReefscapeAuto extends SequentialCommandGroup {
    
    /**
     * Creates a new autonomous routine for the Reefscape game
     */
    public ReefscapeAuto() {
        addCommands(
            // Phase 1: Initialization
            new InstantCommand(() -> {
                // Reset encoders to establish a new reference point
                Robot.m_driveSubsystem.resetEncoders();
                
                // Set vision system to ball detection mode
                Robot.m_visionSubsystem.setPipeline(0);
                
                // Log start of autonomous routine
                System.out.println("Starting Reefscape autonomous routine");
            }),
            
            // Phase 2: Move into field
            // Drive forward 1 meter at 40% power to enter field
            new Drivetrain_GyroStraight(1.0, 0.4),
            
            // Phase 3: Scan for game pieces
            // Turn 45 degrees to scan the field
            new Drivetrain_GyroTurn(45),
            
            // Brief pause to stabilize after turning
            new WaitCommand(0.2),
            
            // Phase 4: Locate and collect a game piece
            // Activate vision tracking to find and collect a ball
            new BallTrackingCommand(),
            
            // Pause to ensure arm is safely stowed after collection
            new WaitCommand(0.5),
            
            // Phase 5: Return to scoring position
            // Turn 180 degrees to face the scoring zone
            new Drivetrain_GyroTurn(180),
            
            // Pause to stabilize after turning
            new WaitCommand(0.2),
            
            // Drive to scoring position (1.2 meters at 50% power)
            new Drivetrain_GyroStraight(1.2, 0.5),
            
            // Fine-tune alignment with scoring target
            new Drivetrain_GyroTurn(-10),
            
            // Pause before scoring for stability
            new WaitCommand(0.3),
            
            // Phase 6: Score the game piece
            // Execute scoring sequence with the ball arm
            new ScoreSequence(Robot.m_ballArmSubsystem),
            
            // Phase 7: Finalization
            // Return arm to home position for teleop
            new InstantCommand(() -> {
                Robot.m_ballArmSubsystem.homeArm();
                System.out.println("Autonomous routine complete");
            })
        );
    }
}
