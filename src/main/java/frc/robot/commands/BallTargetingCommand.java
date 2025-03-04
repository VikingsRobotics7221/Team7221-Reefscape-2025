// src/main/java/frc/robot/commands/BallTargetingCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * BallTargetingCommand - ULTIMATE BALL TRACKING SYSTEM
 * 
 * This is Team 7221's automated targeting system that:
 * 1. Uses vision to locate balls
 * 2. Drives to optimal collection position 
 * 3. Deploys arm and collects
 * 4. All in one smooth operation!
 * 
 * coded by paysean
 */
public class BallTargetingCommand extends Command {
    
    // ========== STATE MACHINE SETUP ==========
    private enum TargetingState {
        SEARCHING,   // Looking for a ball
        ALIGNING,    // Turning to face ball
        APPROACHING, // Driving to ball
        COLLECTING,  // Grabbing the ball
        RETURNING,   // Going back to ready position
        COMPLETE     // All done!
    }
    
    // ========== SUBSYSTEMS ==========
    private final DriveSubsystem m_drive;
    private final BallArmSubsystem m_arm;
    private final VisionSubsystem m_vision;
    
    // ========== TRACKING VARIABLES ==========
    private TargetingState m_state = TargetingState.SEARCHING;
    private long m_stateStartTime = 0;
    private double m_targetYaw = 0.0;
    private double m_targetDistance = 0.0;
    private boolean m_hasTimedOut = false;
    
    // ========== STATE TIMEOUT LIMITS (ms) ==========
    private static final long SEARCH_TIMEOUT = 5000;    // 5 seconds max search
    private static final long ALIGN_TIMEOUT = 2000;     // 2 seconds max alignment
    private static final long APPROACH_TIMEOUT = 3000;  // 3 seconds max approach
    private static final long COLLECT_TIMEOUT = 2000;   // 2 seconds max collection
    private static final long RETURN_TIMEOUT = 2000;    // 2 seconds max return
    
    // ========== ASCII ART FTW ==========
    private static final String[] STATE_ART = {
        "  _____  \n |     | \n | 👁️👁️ | SEARCHING\n |_____| ",
        "  _____  \n |     | \n | 👀  | ALIGNING\n |_____| ",
        "  _____  \n |     | \n | 🚗💨 | APPROACHING\n |_____| ",
        "  _____  \n |     | \n | 🤲  | COLLECTING\n |_____| ",
        "  _____  \n |     | \n | 🏠  | RETURNING\n |_____| ",
        "  _____  \n |     | \n | 🎯  | COMPLETE\n |_____| "
    };
    
    /**
     * Creates a new BallTargetingCommand - THE BALL HUNTER 9000!
     */
    public BallTargetingCommand() {
        // Get references to our subsystems from Robot
        m_drive = Robot.m_driveSubsystem;
        m_arm = Robot.m_ballArmSubsystem;
        m_vision = Robot.m_visionSubsystem;
        
        // THIS COMMAND NEEDS CONTROL OF THESE SUBSYSTEMS
        addRequirements(m_drive, m_arm);
        
        System.out.println(" ___________________________ ");
        System.out.println("|                           |");
        System.out.println("| BALL TARGETING ACTIVATED! |");
        System.out.println("|___________________________|");
    }
    
    @Override
    public void initialize() {
        // Start in SEARCHING state
        changeState(TargetingState.SEARCHING);
        
        // Switch vision pipeline to ball detection
        m_vision.setPipeline(0); // 0 = Ball tracking pipeline
        
        // Make sure arm is in home position
        m_arm.homeArm();
        
        // Switch to precision drive mode for smoother targeting
        m_drive.enablePrecisionMode();
        
        // Disable manual control while targeting system is active
        Robot.manualDriveControl = false;
    }
    
    @Override
    public void execute() {
        // Update dashboard with our current state
        SmartDashboard.putString("Targeting State", m_state.toString());
        
        // Check for timeout in current state
        checkForTimeout();
        
        // State machine logic - run the current state's code
        switch (m_state) {
            case SEARCHING:
                executeSearching();
                break;
                
            case ALIGNING:
                executeAligning();
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
                // Do nothing, waiting for command to end
                break;
        }
    }
    
    /**
     * SEARCHING state - Spin around looking for balls
     */
    private void executeSearching() {
        if (m_vision.getHasTarget()) {
            // FOUND SOMETHING! Get target info
            m_targetYaw = m_vision.getBestTarget().getYaw();
            m_targetDistance = m_vision.getTargetDistance();
            
            System.out.println("🔍 BALL DETECTED! Yaw: " + m_targetYaw + 
                               "° Distance: " + m_targetDistance + "m");
            
            // Move to alignment state
            changeState(TargetingState.ALIGNING);
        } else {
            // Still searching - rotate slowly to scan for balls
            m_drive.arcadeDrive(0, 0.25); // Spin at 25% power
        }
    }
    
    /**
     * ALIGNING state - Turn to face the ball
     */
    private void executeAligning() {
        if (m_vision.getHasTarget()) {
            // Update targeting data
            m_targetYaw = m_vision.getBestTarget().getYaw();
            m_targetDistance = m_vision.getTargetDistance();
            
            // Calculate turn power proportional to how far off we are
            // PID would be better, but P control is good enough here
            double turnPower = m_targetYaw * 0.015; // P gain
            
            // Limit max turn power for smooth movement
            turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
            
            // Apply turn power
            m_drive.arcadeDrive(0, turnPower);
            
            // Check if we're aligned with target
            if (Math.abs(m_targetYaw) < 3.0) { // Within 3 degrees
                // We're aligned! Move to approach
                changeState(TargetingState.APPROACHING);
            }
        } else {
            // We lost the target!
            m_drive.arcadeDrive(0, 0); // Stop turning
            changeState(TargetingState.SEARCHING); // Go back to search
        }
    }
    
    /**
     * APPROACHING state - Drive toward the ball
     */
    private void executeApproaching() {
        if (m_vision.getHasTarget()) {
            // Update targeting data
            m_targetYaw = m_vision.getBestTarget().getYaw();
            m_targetDistance = m_vision.getTargetDistance();
            
            // Calculate turn correction to stay aligned while driving
            double turnPower = m_targetYaw * 0.015; // P gain
            
            // Calculate drive speed - slow down as we get closer
            double forwardSpeed = 0.4; // Base speed
            
            if (m_targetDistance < 1.0) {
                // Slow down when we're under 1 meter
                forwardSpeed = 0.3;
            }
            
            if (m_targetDistance < 0.5) {
                // Really slow when very close
                forwardSpeed = 0.2;
            }
            
            // Drive toward ball
            m_drive.arcadeDrive(forwardSpeed, turnPower);
            
            // Check if we're close enough to collect
            if (m_targetDistance < 0.3) { // 30cm
                m_drive.arcadeDrive(0, 0); // Full stop
                changeState(TargetingState.COLLECTING);
            }
        } else {
            // Lost sight of the ball!
            m_drive.arcadeDrive(0, 0); // Stop
            
            // Did we lose it because we're too close?
            if (m_targetDistance < 0.4) {
                // Probably close enough - try to collect
                changeState(TargetingState.COLLECTING);
            } else {
                // Actually lost it - go back to searching
                changeState(TargetingState.SEARCHING);
            }
        }
    }
    
    /**
     * COLLECTING state - Grab the ball with the arm
     */
    private void executeCollecting() {
        // Stop the drivetrain
        m_drive.arcadeDrive(0, 0);
        
        // First frame in this state?
        long timeInState = System.currentTimeMillis() - m_stateStartTime;
        if (timeInState < 100) { // Just entered this state
            // Deploy the arm
            m_arm.pickupPosition();
            
            // Start the intake
            m_arm.setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
            
            System.out.println("🦾 ARM DEPLOYED! NOMNOM TIME!");
        }
        
        // Do we have the ball yet?
        if (m_arm.hasBall()) {
            // YES! Ball acquired
            m_arm.setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
            changeState(TargetingState.RETURNING);
            
            System.out.println("✅ BALL ACQUIRED! MISSION SUCCESSFUL!");
        }
    }
    
    /**
     * RETURNING state - Put everything back in travel position
     */
    private void executeReturning() {
        // Return arm to safe position
        m_arm.homeArm();
        
        // Return to original orientation (if appropriate)
        double currentAngle = m_drive.getGyroAngle();
        
        // If we're way off from our original heading, turn back
        if (Math.abs(currentAngle) > 45) {
            double turnPower = -currentAngle * 0.01;
            
            // Limit turn power
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
            
            m_drive.arcadeDrive(0, turnPower);
            
            // Are we close enough to original heading?
            if (Math.abs(currentAngle) < 5) {
                m_drive.arcadeDrive(0, 0);
                changeState(TargetingState.COMPLETE);
            }
        } else {
            // We didn't turn much, just finish
            changeState(TargetingState.COMPLETE);
        }
    }
    
    /**
     * Change to a new state
     */
    private void changeState(TargetingState newState) {
        // Record when we entered this state
        m_stateStartTime = System.currentTimeMillis();
        
        // Update the state
        m_state = newState;
        
        // Print cool status message with ASCII art
        System.out.println("\n" + STATE_ART[m_state.ordinal()]);
    }
    
    /**
     * Check if current state has timed out
     */
    private void checkForTimeout() {
        long timeInState = System.currentTimeMillis() - m_stateStartTime;
        long timeoutLimit = 0;
        
        // Set timeout based on current state
        switch (m_state) {
            case SEARCHING:
                timeoutLimit = SEARCH_TIMEOUT;
                break;
            case ALIGNING:
                timeoutLimit = ALIGN_TIMEOUT;
                break;
            case APPROACHING:
                timeoutLimit = APPROACH_TIMEOUT;
                break;
            case COLLECTING:
                timeoutLimit = COLLECT_TIMEOUT;
                break;
            case RETURNING:
                timeoutLimit = RETURN_TIMEOUT;
                break;
            case COMPLETE:
                return; // No timeout for COMPLETE state
        }
        
        // Check if we've timed out
        if (timeInState > timeoutLimit) {
            System.out.println("⚠️ STATE TIMEOUT: " + m_state + 
                               " after " + timeInState + "ms");
            
            // Handle timeout based on state
            switch (m_state) {
                case COLLECTING:
                    // If collecting times out, stop intake and return
                    m_arm.setGripper(0);
                    m_arm.homeArm();
                    changeState(TargetingState.RETURNING);
                    break;
                    
                case RETURNING:
                    // If returning times out, just finish
                    changeState(TargetingState.COMPLETE);
                    break;
                    
                default:
                    // For other states, signal timeout and finish
                    m_hasTimedOut = true;
                    changeState(TargetingState.COMPLETE);
                    break;
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop everything
        m_drive.arcadeDrive(0, 0);
        
        // Return arm to safe position
        m_arm.homeArm();
        
        // Restore normal driving mode
        m_drive.disableDriveModes();
        
        // Re-enable manual control
        Robot.manualDriveControl = true;
        
        // Status message
        if (interrupted) {
            System.out.println("🛑 BALL TARGETING INTERRUPTED BY DRIVER");
        } else if (m_hasTimedOut) {
            System.out.println("⏱️ BALL TARGETING TIMED OUT");
        } else {
            System.out.println("🎯 BALL TARGETING COMPLETED SUCCESSFULLY");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Only finish when state machine reaches COMPLETE
        return m_state == TargetingState.COMPLETE;
    }
}
