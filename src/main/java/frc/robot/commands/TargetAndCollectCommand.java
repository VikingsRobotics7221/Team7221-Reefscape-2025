// src/main/java/frc/robot/commands/TargetAndCollectCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * TargetAndCollectCommand - THE ULTIMATE BALL HUNTING SYSTEM
 * 
 * This command uses vision targeting to find balls, drive to them, and collect them
 * in a smooth, single operation. It's like a heat-seeking missile for balls!
 * 
 * 
 *   [CAMERA] --> [VISION PROCESSING] --> [DRIVETRAIN] --> [BALL ARM]
 *       |             |                      |               |
 *       v             v                      v               v
 *   SEES BALL     CALCULATES PATH       MOVES ROBOT     GRABS BALL
 * 
 * coded by paysean
 */
public class TargetAndCollectCommand extends Command {
    
    // The subsystems we need for the EPIC BALL HUNT
    private final DriveSubsystem m_driveSubsystem;
    private final BallArmSubsystem m_ballArmSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    
    // State machine variables
    private enum State {
        SEARCHING,    // Looking for balls
        APPROACHING,  // Driving toward ball
        COLLECTING,   // Activating arm to grab
        RETURNING,    // Going back to home position
        COMPLETE      // Mission accomplished!
    }
    
    private State m_currentState = State.SEARCHING;
    private double m_ballDistance = 0.0;
    private double m_ballAngle = 0.0;
    private long m_stateStartTime = 0;
    private boolean m_hasTimeout = false;
    
    // ASCII ART STATE DISPLAY FOR DEBUGGING!!!
    private final String[] STATE_ASCII = {
        "  _.---._  \n /       \\ \n|  SEARCH  |\n \\       / \n  '-._.-'  ",
        "  _.---._  \n /       \\ \n| APPROACH |\n \\       / \n  '-._.-'  ",
        "  _.---._  \n /       \\ \n| COLLECT! |\n \\       / \n  '-._.-'  ",
        "  _.---._  \n /       \\ \n| RETURN   |\n \\       / \n  '-._.-'  ",
        "  _.---._  \n /       \\ \n| COMPLETE!|\n \\       / \n  '-._.-'  "
    };
    
    /**
     * Creates a new TargetAndCollectCommand - THE BALL HUNTER!!!
     */
    public TargetAndCollectCommand() {
        // Get the subsystems from Robot
        m_driveSubsystem = Robot.m_driveSubsystem;
        m_ballArmSubsystem = Robot.m_ballArmSubsystem;
        m_visionSubsystem = Robot.m_visionSubsystem;
        
        // We need control of these subsystems
        addRequirements(m_driveSubsystem, m_ballArmSubsystem);
        
        System.out.println(" ___________________________________");
        System.out.println("| BALL TARGETING SYSTEM ACTIVATED! |");
        System.out.println("|___________________________________|");
    }
    
    @Override
    public void initialize() {
        // Reset state machine to beginning
        m_currentState = State.SEARCHING;
        m_stateStartTime = System.currentTimeMillis();
        
        // Put arm in upright position for better camera view
        m_ballArmSubsystem.setArmPosition(Constants.BALL_ARM_HOME_POSITION);
        
        // Make sure the drive system is in precision mode for smoother approaches
        m_driveSubsystem.enablePrecisionMode();
        
        // Disable manual control during autonomous ball collection
        Robot.manualDriveControl = false;
        
        // Debug output
        System.out.println(STATE_ASCII[m_currentState.ordinal()]);
        System.out.println(">> SCANNING FOR BALLS... >>");
    }
    
    @Override
    public void execute() {
        // Update dashboard with current state
        SmartDashboard.putString("Ball Hunt State", m_currentState.toString());
        
        // State machine for ball hunting
        switch (m_currentState) {
            case SEARCHING:
                executeSearching();
                break;
                
            case APPROACHING:
                executeApproaching();
                break;
                
            case COLLECTING:
                executeCollecting();
                break;
                
            case RETURNING:
                executeReturning();
                break;
                
            case COMPLETE:
                // Nothing to do, just wait for command to end
                break;
        }
        
        // Check for timeouts in each state
        checkStateTimeout();
    }
    
    /**
     * SEARCHING state - Look for balls
     */
    private void executeSearching() {
        // If vision system has found a ball target
        if (m_visionSubsystem.getHasTarget()) {
            // Get the ball data
            m_ballDistance = m_visionSubsystem.getTargetDistance();
            m_ballAngle = m_visionSubsystem.getBestTarget().getYaw();
            
            // Move to approaching state
            transitionToState(State.APPROACHING);
            
            System.out.println(">> BALL DETECTED AT DISTANCE: " + m_ballDistance + "m");
            System.out.println(">> ANGLE TO BALL: " + m_ballAngle + "°");
            
        } else {
            // No ball found yet, keep spinning slowly to look around
            m_driveSubsystem.arcadeDrive(0, 0.2);
        }
    }
    
    /**
     * APPROACHING state - Drive toward the ball
     */
    private void executeApproaching() {
        // If we still see the ball, update tracking data
        if (m_visionSubsystem.getHasTarget()) {
            m_ballDistance = m_visionSubsystem.getTargetDistance();
            m_ballAngle = m_visionSubsystem.getBestTarget().getYaw();
            
            // Calculate drive parameters
            double turnPower = -m_ballAngle * Constants.TRACKED_TAG_ROATION_KP;
            
            // Calculate forward power - slow down as we get closer
            double forwardPower = 0.4;
            if (m_ballDistance < 1.0) {
                forwardPower = 0.3;
            }
            if (m_ballDistance < 0.5) {
                forwardPower = 0.2;
            }
            
            // Drive toward ball
            m_driveSubsystem.arcadeDrive(forwardPower, turnPower);
            
            // When we're close enough, switch to collecting
            if (m_ballDistance < Constants.BALL_DETECTION_THRESHOLD_INCHES / 39.37) { // convert inches to meters
                transitionToState(State.COLLECTING);
            }
            
        } else {
            // Lost sight of the ball!
            System.out.println(">> LOST VISUAL ON TARGET!");
            
            // Stop and go back to searching
            m_driveSubsystem.arcadeDrive(0, 0);
            transitionToState(State.SEARCHING);
        }
    }
    
    /**
     * COLLECTING state - Grab the ball
     */
    private void executeCollecting() {
        // First, stop the robot
        m_driveSubsystem.arcadeDrive(0, 0);
        
        // Check if this is the first loop in this state
        long timeInState = System.currentTimeMillis() - m_stateStartTime;
        if (timeInState < 100) {
            // Just entered state, start the collection sequence
            m_ballArmSubsystem.pickupPosition();
            m_ballArmSubsystem.setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
            
            System.out.println(">> ARM DEPLOYED! GRABBING BALL!");
        }
        
        // Check if we've successfully collected a ball
        if (m_ballArmSubsystem.hasBall()) {
            // Got a ball! Switch to returning
            m_ballArmSubsystem.setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
            transitionToState(State.RETURNING);
            
            System.out.println(">> BALL ACQUIRED! RETURNING TO HOME POSITION!");
        }
        
        // If we've been in this state too long without getting a ball
        if (timeInState > 3000) {
            // Timeout - go back to searching
            m_ballArmSubsystem.setGripper(0);
            m_ballArmSubsystem.homeArm();
            transitionToState(State.SEARCHING);
            
            System.out.println(">> COLLECTION TIMEOUT - RETURNING TO SEARCH");
        }
    }
    
    /**
     * RETURNING state - Return to home position or orientation
     */
    private void executeReturning() {
        // First, put the arm back to home position
        m_ballArmSubsystem.homeArm();
        
        // Then, use the gyro to turn back to starting orientation (0 degrees)
        double angleError = -m_driveSubsystem.getGyroAngle();
        double turnPower = angleError * Constants.GYRO_TURN_KP;
        
        // Limit turn power for safety
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
        
        // Apply turning power
        m_driveSubsystem.arcadeDrive(0, turnPower);
        
        // Check if we're close enough to the target angle
        if (Math.abs(angleError) < Constants.TURNING_THRESHOLD_DEGREES) {
            // We're at the right angle, we're done!
            m_driveSubsystem.arcadeDrive(0, 0);
            transitionToState(State.COMPLETE);
            
            System.out.println(">> MISSION COMPLETE! READY FOR DRIVER CONTROL!");
        }
    }
    
    /**
     * Check if we've been in the current state too long
     */
    private void checkStateTimeout() {
        // Only check states that should timeout
        if (m_currentState == State.SEARCHING || m_currentState == State.APPROACHING) {
            long timeInState = System.currentTimeMillis() - m_stateStartTime;
            
            // If we've been in this state too long
            if (timeInState > 10000) { // 10 seconds max per state
                m_hasTimeout = true;
                
                System.out.println(">> STATE TIMEOUT - COMMAND ENDING");
                m_currentState = State.COMPLETE;
            }
        }
    }
    
    /**
     * Change to a new state in our state machine
     */
    private void transitionToState(State newState) {
        m_currentState = newState;
        m_stateStartTime = System.currentTimeMillis();
        
        // Display ASCII art for current state (this is cool, right?!)
        System.out.println(STATE_ASCII[m_currentState.ordinal()]);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop everything
        m_driveSubsystem.arcadeDrive(0, 0);
        m_ballArmSubsystem.homeArm();
        
        // Return to normal drive mode
        m_driveSubsystem.disableDriveModes();
        
        // Re-enable manual control
        Robot.manualDriveControl = true;
        
        if (interrupted) {
            System.out.println(">> BALL TARGETING INTERRUPTED BY DRIVER");
        } else if (m_hasTimeout) {
            System.out.println(">> BALL TARGETING ENDED DUE TO TIMEOUT");
        } else {
            System.out.println(">> BALL TARGETING COMPLETED SUCCESSFULLY");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Only finish if we've completed the entire sequence or timed out
        return m_currentState == State.COMPLETE;
    }
}
