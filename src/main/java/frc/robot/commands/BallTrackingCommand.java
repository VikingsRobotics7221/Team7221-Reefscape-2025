// src/main/java/frc/robot/commands/BallTrackingCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * BallTrackingCommand - THE ULTIMATE AUTO BALL HUNTER!
 * 
 * This awesome command uses the vision system to find, approach, and collect
 * balls automatically, making us UNSTOPPABLE in Reefscape.
 * 
 * coded by paysean
 */
public class BallTrackingCommand extends Command {
    
    // State machine for tracking our progress
    private enum TrackingState {
        SEARCHING,    // Looking for balls
        ALIGNING,     // Turning to face the ball
        APPROACHING,  // Driving to the ball
        COLLECTING,   // Grabbing the ball
        RETURNING,    // Going back to ready position
        COMPLETE      // All done!
    }
    
    // ===== SUBSYSTEMS WE NEED =====
    private final DriveSubsystem m_drive;
    private final BallArmSubsystem m_arm;
    private final VisionSubsystem m_vision;
    
    // ===== TRACKING VARIABLES =====
    private TrackingState m_state = TrackingState.SEARCHING;
    private long m_stateStartTime = 0;
    private double m_targetYaw = 0.0;
    private double m_targetDistance = 0.0;
    private boolean m_hasTimedOut = false;
    
    // ===== TIMEOUT LIMITS =====
    private static final long SEARCH_TIMEOUT = 5000;    // 5 sec timeout
    private static final long ALIGN_TIMEOUT = 2000;     // 2 sec timeout
    private static final long APPROACH_TIMEOUT = 3000;  // 3 sec timeout
    private static final long COLLECT_TIMEOUT = 2000;   // 2 sec timeout
    private static final long RETURN_TIMEOUT = 2000;    // 2 sec timeout
    
    /**
     * Creates a new BallTrackingCommand - The ultimate ball finder!
     */
    public BallTrackingCommand() {
        // Get subsystem references from Robot
        m_drive = Robot.m_driveSubsystem;
        m_arm = Robot.m_ballArmSubsystem;
        m_vision = Robot.m_visionSubsystem;
        
        // This command NEEDS these subsystems
        addRequirements(m_drive, m_arm);
        
        System.out.println("🔥🔥🔥 BALL TRACKING SYSTEM ONLINE 🔥🔥🔥");
    }
    
    @Override
    public void initialize() {
        // Start in SEARCHING state
        changeState(TrackingState.SEARCHING);
        
        // Make sure vision is set to ball detection mode
        m_vision.setPipeline(0);  // Pipeline 0 = Ball detection
        
        // Put arm in safe position
        m_arm.homeArm();
        
        // Use precision driving for smoother approach
        m_drive.enablePrecisionMode();
        
        // Disable manual control while we're auto-tracking
        Robot.manualDriveControl = false;
    }
    
    @Override
    public void execute() {
        // Update dashboard with current state
        SmartDashboard.putString("Tracking State", m_state.toString());
        
        // Check for timeouts
        checkForTimeout();
        
        // Run the state machine
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
                // Nothing to do, just waiting to end
                break;
        }
    }
    
    /**
     * SEARCHING state - Spin around looking for balls
     */
    private void executeSearching() {
        // Check if vision system has found a target
        if (m_vision.getHasTarget()) {
            // We found something! Get target info
            m_targetYaw = m_vision.getBestTarget().getYaw();
            m_targetDistance = m_vision.getTargetDistance();
            
            System.out.println("🔍 BALL SPOTTED! Yaw: " + m_targetYaw + 
                              "° Distance: " + m_targetDistance + "m");
            
            // Move to alignment state
            changeState(TrackingState.ALIGNING);
        } else {
            // No target yet, keep searching - rotate slowly
            m_drive.arcadeDrive(0, 0.25);  // 25% rotation power
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
            
            // Calculate turn power based on how far off we are
            double turnPower = m_targetYaw * 0.015;  // P control gain
            
            // Limit max turn power for smooth movement
            turnPower = Math.max(-0.5, Math.min(0.5, turnPower));
            
            // Apply turn power
            m_drive.arcadeDrive(0, turnPower);
            
            // Check if we're aligned (within 3 degrees)
            if (Math.abs(m_targetYaw) < 3.0) {
                // We're aligned! Move to approach state
                changeState(TrackingState.APPROACHING);
            }
        } else {
            // Lost sight of target!
            m_drive.arcadeDrive(0, 0);  // Stop turning
            changeState(TrackingState.SEARCHING);  // Go back to search
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
            
            // Calculate turn correction to stay aligned
            double turnPower = m_targetYaw * 0.015;  // P control gain
            
            // Calculate forward speed - slow down as we get closer
            double forwardSpeed = 0.4;  // Base speed
            
            if (m_targetDistance < 1.0) {
                // Slow down when closer than 1 meter
                forwardSpeed = 0.3;
            }
            
            if (m_targetDistance < 0.5) {
                // Really slow when very close
                forwardSpeed = 0.2;
            }
            
            // Drive toward ball
            m_drive.arcadeDrive(forwardSpeed, turnPower);
            
            // Check if we're close enough to collect
            if (m_targetDistance < Constants.BALL_DETECTION_THRESHOLD_INCHES / 39.37) { // convert inches to meters
                m_drive.arcadeDrive(0, 0);  // Stop
                changeState(TrackingState.COLLECTING);
            }
            
        } else {
            // Lost sight of the ball!
            m_drive.arcadeDrive(0, 0);  // Stop
            
            // Did we lose it because we're too close?
            if (m_targetDistance < 0.4) {
                // Probably close enough - try to collect
                changeState(TrackingState.COLLECTING);
            } else {
                // Actually lost it - go back to searching
                changeState(TrackingState.SEARCHING);
            }
        }
    }
    
    /**
     * COLLECTING state - Grab the ball with the arm
     */
    private void executeCollecting() {
        // Stop the drivetrain
        m_drive.arcadeDrive(0, 0);
        
        // Just entered this state?
        long timeInState = System.currentTimeMillis() - m_stateStartTime;
        if (timeInState < 100) {  // First 100ms in this state
            // Deploy the arm
            m_arm.pickupPosition();
            
            // Start the intake
            m_arm.setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
            
            System.out.println("🦾 ARM DEPLOYED! GRABBING BALL!");
        }
        
        // Did we get the ball?
        if (m_arm.hasBall()) {
            // Success! Ball acquired
            m_arm.setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
            changeState(TrackingState.RETURNING);
            
            System.out.println("✅ BALL ACQUIRED! MISSION SUCCESSFUL!");
        }
        
        // If we've been in this state too long without getting a ball
        if (timeInState > 3000) {
            // Timeout - go back to searching
            m_arm.setGripper(0);
            m_arm.homeArm();
            changeState(TrackingState.SEARCHING);
            
            System.out.println("⏱️ COLLECTION TIMEOUT - RETURNING TO SEARCH");
        }
    }
    
    /**
     * RETURNING state - Return to home position or orientation
     */
    private void executeReturning() {
        // First, put the arm back to home position
        m_arm.homeArm();
        
        // Then, use the gyro to turn back to starting orientation (0 degrees)
        double angleError = -m_drive.getGyroAngle();
        double turnPower = angleError * Constants.GYRO_TURN_KP;
        
        // Limit turn power for safety
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
        
        // Apply turning power
        m_drive.arcadeDrive(0, turnPower);
        
        // Check if we're close enough to the target angle
        if (Math.abs(angleError) < Constants.TURNING_THRESHOLD_DEGREES) {
            // We're at the right angle, we're done!
            m_drive.arcadeDrive(0, 0);
            changeState(TrackingState.COMPLETE);
            
            System.out.println("🎯 MISSION COMPLETE! READY FOR DRIVER CONTROL!");
        }
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
                return;  // No timeout for COMPLETE state
        }
        
        // Check if we've timed out
        if (timeInState > timeoutLimit) {
            System.out.println("⚠️ STATE TIMEOUT: " + m_state);
            
            // Handle timeout based on state
            switch (m_state) {
                case COLLECTING:
                    // If collecting times out, stop intake and return
                    m_arm.setGripper(0);
                    m_arm.homeArm();
                    changeState(TrackingState.RETURNING);
                    break;
                    
                case RETURNING:
                    // If returning times out, just finish
                    changeState(TrackingState.COMPLETE);
                    break;
                    
                default:
                    // For other states, signal timeout and finish
                    m_hasTimedOut = true;
                    changeState(TrackingState.COMPLETE);
                    break;
            }
        }
    }
    
    /**
     * Change to a new state in our state machine
     */
    private void changeState(TrackingState newState) {
        // Record when we entered this state
        m_stateStartTime = System.currentTimeMillis();
        
        // Update the state
        m_state = newState;
        
        // Log state change
        System.out.println("🔄 TRACKING STATE: " + m_state);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop everything
        m_drive.arcadeDrive(0, 0);
        
        // Return arm to safe position
        m_arm.homeArm();
        
        // Return to normal driving mode
        m_drive.disableDriveModes();
        
        // Re-enable manual control
        Robot.manualDriveControl = true;
        
        // Status message
        if (interrupted) {
            System.out.println("🛑 BALL TRACKING INTERRUPTED");
        } else if (m_hasTimedOut) {
            System.out.println("⏱️ BALL TRACKING TIMED OUT");
        } else {
            System.out.println("🎯 BALL TRACKING COMPLETED SUCCESSFULLY");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Only finish when we reach COMPLETE state
        return m_state == TrackingState.COMPLETE;
    }
}
