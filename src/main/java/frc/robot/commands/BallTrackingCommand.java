package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;

/**
 * ┌───────────────────────────────────────────────────────────┐
 * │  BALL TRACKING COMMAND - VISION-BASED BALL ACQUISITION    │
 * │  Autonomous ball detection and collection system          │
 * └───────────────────────────────────────────────────────────┘
 */
public class BallTrackingCommand extends Command {
    
    // State machine states
    private enum TrackingState {
        INITIALIZING,  // Setting up systems
        SCANNING,      // Looking for balls
        TARGETING,     // Ball found, orienting toward it
        APPROACHING,   // Moving toward ball
        COLLECTING,    // Deploying arm to collect ball
        SECURING,      // Confirming successful collection
        RETURNING,     // Going back to starting position
        COMPLETE       // All done
    }
    
    // Timers and tracking
    private final Timer m_stateTimer = new Timer();
    private final Timer m_commandTimer = new Timer();
    private TrackingState m_state = TrackingState.INITIALIZING;
    private double m_initialAngle = 0.0;
    private int m_scanDirection = 1;
    private int m_scanCount = 0;
    private double m_lastYaw = 0.0;
    private double m_lastDistance = 0.0;
    private boolean m_hasTarget = false;
    
    // Speed constants
    private static final double SCAN_SPEED = 0.3;
    private static final double APPROACH_SPEED = 0.4;
    private static final double TURN_SPEED = 0.3;
    private static final double COLLECTION_DISTANCE = 0.5; // meters
    
    /**
     * Creates a new BallTrackingCommand that automatically
     * uses the subsystems from Robot.java
     */
    public BallTrackingCommand() {
        // Add subsystem requirements
        addRequirements(Robot.m_driveSubsystem, Robot.m_ballArmSubsystem);
    }
    
    @Override
    public void initialize() {
        // Start in INITIALIZING state
        transitionToState(TrackingState.INITIALIZING);
        
        // Reset and start timers
        m_commandTimer.reset();
        m_commandTimer.start();
        m_stateTimer.reset();
        m_stateTimer.start();
        
        // Store initial position for return
        m_initialAngle = 0; // We don't have a gyro, so we'll track with encoders
        
        // Random scan direction
        m_scanDirection = Math.random() > 0.5 ? 1 : -1;
        
        // Set vision system to ball detection mode
        Robot.m_visionSubsystem.setPipeline(Constants.Vision.BALL_PIPELINE_INDEX);
        
        System.out.println(">> BALL TRACKING INITIALIZED");
        SmartDashboard.putBoolean("Ball Tracking Active", true);
    }
    
    @Override
    public void execute() {
        // Update dashboard with current state
        SmartDashboard.putString("Tracking State", m_state.toString());
        
        // Check if vision system has a target
        m_hasTarget = Robot.m_visionSubsystem.getHasTarget();
        
        // Run appropriate state logic
        switch (m_state) {
            case INITIALIZING:
                executeInitializing();
                break;
                
            case SCANNING:
                executeScanning();
                break;
                
            case TARGETING:
                executeTargeting();
                break;
                
            case APPROACHING:
                executeApproaching();
                break;
                
            case COLLECTING:
                executeCollecting();
                break;
                
            case SECURING:
                executeSecuring();
                break;
                
            case RETURNING:
                executeReturning();
                break;
                
            case COMPLETE:
                // Nothing to do in complete state
                break;
        }
        
        // Check for timeouts
        checkForTimeout();
    }
    
    /**
     * INITIALIZING state - prepare for ball hunting
     */
    private void executeInitializing() {
        // Put arm in home position for better vision
        Robot.m_ballArmSubsystem.homeArm();
        
        // Wait briefly for initialization
        if (m_stateTimer.get() > 0.5) {
            transitionToState(TrackingState.SCANNING);
        }
    }
    
    /**
     * SCANNING state - rotate to find a ball
     */
    private void executeScanning() {
        if (m_hasTarget) {
            // Target found! Get data and move to targeting
            m_lastYaw = Robot.m_visionSubsystem.getBestTarget().getYaw();
            m_lastDistance = Robot.m_visionSubsystem.getTargetDistance();
            transitionToState(TrackingState.TARGETING);
            return;
        }
        
        // No target yet, keep scanning
        // Use tank drive to rotate in place
        Robot.m_driveSubsystem.tankDrive(
            SCAN_SPEED * m_scanDirection, 
            -SCAN_SPEED * m_scanDirection
        );
        
        // Change direction if we've been scanning too long
        if (m_stateTimer.get() > 4.0 && m_scanCount < 2) {
            m_scanDirection *= -1;
            m_scanCount++;
            System.out.println(">> Changing scan direction");
            
            // Reset state timer but stay in scanning state
            m_stateTimer.reset();
        }
    }
    
    /**
     * TARGETING state - align with the detected ball
     */
    private void executeTargeting() {
        if (m_hasTarget) {
            // Update target information
            m_lastYaw = Robot.m_visionSubsystem.getBestTarget().getYaw();
            m_lastDistance = Robot.m_visionSubsystem.getTargetDistance();
        } else if (m_stateTimer.get() > 0.5) {
            // Lost target
            transitionToState(TrackingState.SCANNING);
            return;
        }
        
        // Calculate turn direction based on yaw
        // Negative yaw = target to left, positive = target to right
        double turnPower = m_lastYaw * 0.03; // Proportional control
        
        // Limit turn power
        turnPower = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, turnPower));
        
        // Apply turn with tank drive
        Robot.m_driveSubsystem.tankDrive(-turnPower, turnPower);
        
        // Check if we're aligned
        if (Math.abs(m_lastYaw) < 3.0) {
            transitionToState(TrackingState.APPROACHING);
        }
    }
    
    /**
     * APPROACHING state - move toward the ball
     */
    private void executeApproaching() {
        if (m_hasTarget) {
            // Update target information
            m_lastYaw = Robot.m_visionSubsystem.getBestTarget().getYaw();
            m_lastDistance = Robot.m_visionSubsystem.getTargetDistance();
            
            // Calculate approach speed based on distance
            double speed = Math.min(APPROACH_SPEED, m_lastDistance * 0.5);
            speed = Math.max(0.2, speed); // Minimum speed to keep moving
            
            // Calculate small turn correction to stay aligned
            double turnAdjustment = m_lastYaw * 0.02;
            
            // Drive with arcade control
            Robot.m_driveSubsystem.arcadeDrive(speed, turnAdjustment);
            
            // If we're close enough, move to collection
            if (m_lastDistance < COLLECTION_DISTANCE) {
                transitionToState(TrackingState.COLLECTING);
            }
        } else {
            // Lost target, stop and go back to scanning
            Robot.m_driveSubsystem.arcadeDrive(0, 0);
            transitionToState(TrackingState.SCANNING);
        }
    }
    
    /**
     * COLLECTING state - use arm to get the ball
     */
    private void executeCollecting() {
        // Stop driving
        Robot.m_driveSubsystem.arcadeDrive(0, 0);
        
        // Move arm to pickup position
        Robot.m_ballArmSubsystem.pickupPosition();
        
        // Run intake if arm is near pickup position
        if (m_stateTimer.get() > 0.5) {
            Robot.m_ballArmSubsystem.setGripper(Constants.BallArm.GRIPPER_INTAKE_SPEED);
        }
        
        // Check if we got the ball
        if (Robot.m_ballArmSubsystem.hasBall()) {
            transitionToState(TrackingState.SECURING);
        }
        
        // If taking too long, try driving forward a bit
        if (m_stateTimer.get() > 2.0 && m_stateTimer.get() < 2.5) {
            Robot.m_driveSubsystem.arcadeDrive(0.2, 0);
        }
        
        // If still no ball after timeout, give up
        if (m_stateTimer.get() > 3.0) {
            transitionToState(TrackingState.SECURING);
        }
    }
    
    /**
     * SECURING state - secure ball and prepare to return
     */
    private void executeSecuring() {
        // Stop driving
        Robot.m_driveSubsystem.arcadeDrive(0, 0);
        
        // Set gripper to holding power if we have a ball
        if (Robot.m_ballArmSubsystem.hasBall()) {
            Robot.m_ballArmSubsystem.setGripper(Constants.BallArm.GRIPPER_HOLD_SPEED);
            System.out.println(">> Ball secured!");
        } else {
            // No ball, stop intake
            Robot.m_ballArmSubsystem.setGripper(0);
            System.out.println(">> No ball detected");
        }
        
        // Return arm to home position
        Robot.m_ballArmSubsystem.homeArm();
        
        // Wait for arm to start moving, then transition
        if (m_stateTimer.get() > 0.5) {
            transitionToState(TrackingState.RETURNING);
        }
    }
    
    /**
     * RETURNING state - return to starting position
     */
    private void executeReturning() {
        // We don't have a gyro, so we'll just drive back a bit
        double returnSpeed = -0.3; // Slow reverse
        
        // Apply power for a specific time
        if (m_stateTimer.get() < 1.0) {
            Robot.m_driveSubsystem.arcadeDrive(returnSpeed, 0);
        } else {
            // Stop and complete
            Robot.m_driveSubsystem.arcadeDrive(0, 0);
            transitionToState(TrackingState.COMPLETE);
        }
    }
    
    /**
     * Transition to a new state
     */
    private void transitionToState(TrackingState newState) {
        System.out.println(">> BallTracking: " + m_state + " -> " + newState);
        m_state = newState;
        m_stateTimer.reset();
        m_stateTimer.start();
    }
    
    /**
     * Check for timeout in current state
     */
    private void checkForTimeout() {
        // Define timeout durations for each state
        double timeout;
        
        switch (m_state) {
            case INITIALIZING:
                timeout = 1.0;
                break;
            case SCANNING:
                timeout = 15.0; // Allow for a full scan cycle
                break;
            case TARGETING:
                timeout = 3.0;
                break;
            case APPROACHING:
                timeout = 5.0;
                break;
            case COLLECTING:
                timeout = 4.0;
                break;
            case SECURING:
                timeout = 2.0;
                break;
            case RETURNING:
                timeout = 3.0;
                break;
            case COMPLETE:
                timeout = Double.POSITIVE_INFINITY;
                break;
            default:
                timeout = 5.0;
                break;
        }
        
        // Check if we've timed out
        if (m_stateTimer.get() > timeout) {
            // For scanning, change direction and retry
            if (m_state == TrackingState.SCANNING && m_scanCount < 3) {
                m_scanDirection *= -1;
                m_scanCount++;
                m_stateTimer.reset();
            } else {
                // Move to securing and then return
                if (m_state != TrackingState.SECURING && 
                    m_state != TrackingState.RETURNING && 
                    m_state != TrackingState.COMPLETE) {
                    transitionToState(TrackingState.SECURING);
                } else if (m_state == TrackingState.SECURING) {
                    transitionToState(TrackingState.RETURNING);
                } else if (m_state == TrackingState.RETURNING) {
                    transitionToState(TrackingState.COMPLETE);
                }
            }
        }
        
        // Also check overall command timeout
        if (m_commandTimer.get() > 30.0) {
            transitionToState(TrackingState.COMPLETE);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop everything
        Robot.m_driveSubsystem.arcadeDrive(0, 0);
        
        // If we have a ball, maintain hold
        if (Robot.m_ballArmSubsystem.hasBall()) {
            Robot.m_ballArmSubsystem.setGripper(Constants.BallArm.GRIPPER_HOLD_SPEED);
        } else {
            Robot.m_ballArmSubsystem.setGripper(0);
        }
        
        // Return arm to home position
        Robot.m_ballArmSubsystem.homeArm();
        
        // Stop timers
        m_stateTimer.stop();
        m_commandTimer.stop();
        
        // Report status
        System.out.println(">> BALL TRACKING COMPLETE");
        System.out.println(">> Ball acquired: " + Robot.m_ballArmSubsystem.hasBall());
        
        SmartDashboard.putBoolean("Ball Tracking Active", false);
    }
    
    @Override
    public boolean isFinished() {
        // Only finish when we reach the COMPLETE state
        return m_state == TrackingState.COMPLETE;
    }
}
