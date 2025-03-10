// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * BallTrackingCommand - Tracks and collects balls using vision processing.
 * 
 * This command implements a state machine that:
 * 1. Scans the field to locate balls using the vision system
 * 2. Aligns the robot with detected balls
 * 3. Approaches balls at an appropriate speed
 * 4. Collects balls using the arm subsystem
 * 5. Returns to the starting orientation
 * 
 * The command uses PID control for precise movement and includes
 * timeout handling for robustness in competition environments.
 */
public class BallTrackingCommand extends Command {
    
    // State machine definition
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
    
    // Subsystem references
    private final DriveSubsystem m_drive;
    private final BallArmSubsystem m_arm;
    private final VisionSubsystem m_vision;
    
    // PID controllers for precise movement
    private final PIDController m_rotationPID;
    private final PIDController m_distancePID;
    
    // State tracking
    private TrackingState m_state = TrackingState.INITIALIZING;
    private final Timer m_stateTimer = new Timer();
    private final Timer m_commandTimer = new Timer();
    private double m_initialAngle = 0.0;
    private int m_scanDirection = 1;
    private int m_scanCount = 0;
    private double m_lastYaw = 0.0;
    private double m_lastDistance = 0.0;
    private boolean m_acquiredTarget = false;
    
    // Constants for tracking
    private static final double ROTATION_P = 0.02;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;
    private static final double DISTANCE_P = 0.6;
    private static final double DISTANCE_I = 0.0;
    private static final double DISTANCE_D = 0.0;
    private static final double MAX_ROTATION_SPEED = 0.4;
    private static final double MAX_DRIVE_SPEED = 0.45;
    private static final double SCAN_ROTATION_SPEED = 0.3;
    private static final double ALIGNMENT_THRESHOLD_DEGREES = 3.0;
    private static final double COLLECTION_DISTANCE_METERS = 0.5;
    
    /**
     * Creates a new BallTrackingCommand.
     * 
     * @param driveSubsystem The drive subsystem for robot movement
     * @param visionSubsystem The vision subsystem for ball detection
     * @param ballArmSubsystem The ball arm subsystem for collection
     */
    public BallTrackingCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            BallArmSubsystem ballArmSubsystem) {
        
        m_drive = driveSubsystem;
        m_vision = visionSubsystem;
        m_arm = ballArmSubsystem;
        
        // Configure PID controllers
        m_rotationPID = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        m_rotationPID.setTolerance(ALIGNMENT_THRESHOLD_DEGREES);
        
        m_distancePID = new PIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D);
        m_distancePID.setTolerance(0.1);
        
        // Require control of the drive and arm subsystems
        addRequirements(driveSubsystem, ballArmSubsystem);
        
        System.out.println("Ball tracking command created");
    }
    
    @Override
    public void initialize() {
        // Start in INITIALIZING state
        transitionToState(TrackingState.INITIALIZING);
        
        // Start timers
        m_commandTimer.reset();
        m_commandTimer.start();
        
        // Store initial gyro angle for return
        m_initialAngle = m_drive.getGyroAngle();
        
        // Reset PID controllers
        m_rotationPID.reset();
        m_distancePID.reset();
        
        // Initialize scanning direction
        m_scanDirection = (Math.random() > 0.5) ? 1 : -1;
        
        // Make sure vision is in ball detection mode
        m_vision.setPipeline(0);
        
        System.out.println("Ball tracking initialized");
        SmartDashboard.putBoolean("Ball Tracking Active", true);
    }
   
    @Override
    public void execute() {
        // Update dashboard with current state
        SmartDashboard.putString("Ball Tracking State", m_state.toString());
        SmartDashboard.putNumber("State Time", m_stateTimer.get());
        
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
        
        // Check for timeouts based on state
        checkForTimeout();
    }
    
    /**
     * INITIALIZING state - Set up systems for ball hunting
     */
    private void executeInitializing() {
        // Put arm in home position for better camera view
        m_arm.homeArm();
        
        // Wait briefly to let systems stabilize
        if (m_stateTimer.get() > 0.25) {
            transitionToState(TrackingState.SCANNING);
        }
    }
    
    /**
     * SCANNING state - Rotate to search for balls
     */
    private void executeScanning() {
        // Check if vision system has found a target
        if (m_vision.getHasTarget()) {
            // Get target info
            m_lastYaw = m_vision.getBestTarget().getYaw();
            m_lastDistance = m_vision.getTargetDistance();
            
            System.out.println("Ball detected at " + m_lastYaw + 
                              "° yaw, " + m_lastDistance + "m distance");
            
            // Transition to targeting state
            transitionToState(TrackingState.TARGETING);
            return;
        }
        
        // No target yet, keep searching
        double scanTime = m_stateTimer.get();
        
        // Change direction if we've been scanning too long
        if (scanTime > 3.0 && m_scanCount == 0) {
            m_scanDirection *= -1;
            m_scanCount++;
            System.out.println("Changing scan direction");
        }
        
        // Apply rotation for scanning
        m_drive.tankDrive(
            SCAN_ROTATION_SPEED * m_scanDirection, 
            -SCAN_ROTATION_SPEED * m_scanDirection
        );
        
        // Log status periodically
        if (scanTime > 1.0 && Math.floor(scanTime) == scanTime) {
            System.out.println("Scanning for balls... " + (int)scanTime + "s elapsed");
        }
    }
    
    /**
     * TARGETING state - Orient toward the detected ball
     */
    private void executeTargeting() {
        // Update target info if still visible
        if (m_vision.getHasTarget()) {
            m_lastYaw = m_vision.getBestTarget().getYaw();
            m_lastDistance = m_vision.getTargetDistance();
        } else if (m_stateTimer.get() > 0.5) {
            // Lost sight of target
            System.out.println("Target lost during alignment");
            transitionToState(TrackingState.SCANNING);
            return;
        }
        
        // Calculate turn power using PID
        double turnPower = m_rotationPID.calculate(m_lastYaw, 0);
        
        // Limit turn power
        turnPower = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, turnPower));
        
        // Apply turn power (convert to tank drive)
        m_drive.tankDrive(turnPower, -turnPower);
        
        // Check if we're adequately aligned
        boolean isAligned = Math.abs(m_lastYaw) < ALIGNMENT_THRESHOLD_DEGREES;
        
        // If aligned, move to approach state
        if (isAligned) {
            System.out.println("Aligned with target, beginning approach");
            transitionToState(TrackingState.APPROACHING);
        }
    }
    
    /**
     * APPROACHING state - Drive toward the ball
     */
    private void executeApproaching() {
        // Update target info if still visible
        if (m_vision.getHasTarget()) {
            // Update tracking data
            m_lastYaw = m_vision.getBestTarget().getYaw();
            m_lastDistance = m_vision.getTargetDistance();
            m_acquiredTarget = true;
            
            // Calculate course correction (turn power)
            double turnCorrection = m_rotationPID.calculate(m_lastYaw, 0);
            
            // Limit correction for smooth movement
            turnCorrection = Math.max(-0.2, Math.min(0.2, turnCorrection));
            
            // Calculate forward speed based on distance
            double forwardSpeed = calculateApproachSpeed(m_lastDistance);
            
            // Convert to tank drive values
            double leftPower = forwardSpeed + turnCorrection;
            double rightPower = forwardSpeed - turnCorrection;
            
            // Drive toward target
            m_drive.tankDrive(leftPower, rightPower);
            
            // Check if we're close enough to collect
            if (m_lastDistance < COLLECTION_DISTANCE_METERS) {
                System.out.println("In collection range");
                transitionToState(TrackingState.COLLECTING);
            }
        } else if (m_acquiredTarget) {
            // We had target but lost it
            
            // If we're close to where we last saw target, it may be below camera view
            if (m_lastDistance < 0.5) {
                System.out.println("Target lost but continuing approach");
                
                // Keep driving forward (more slowly)
                m_drive.tankDrive(0.15, 0.15);
                
                // If we've been approaching blind for a bit, try collection
                if (m_stateTimer.get() > 1.0) {
                    System.out.println("Attempting collection at approximate position");
                    transitionToState(TrackingState.COLLECTING);
                }
            } else {
                // Lost target and not close - stop and go back to scanning
                m_drive.tankDrive(0, 0);
                System.out.println("Target lost during approach");
                transitionToState(TrackingState.SCANNING);
            }
        } else {
            // Never acquired target - should not happen, but handle gracefully
            m_drive.tankDrive(0, 0);
            System.out.println("No target acquired in approach state");
            transitionToState(TrackingState.SCANNING);
        }
    }
    
    /**
     * COLLECTING state - Deploy arm to grab the ball
     */
    private void executeCollecting() {
        // Stop the drivetrain
        m_drive.tankDrive(0, 0);
        
        // First moment in this state? Deploy the arm!
        if (m_stateTimer.get() < 0.1) {
            // Deploy arm to pickup position
            m_arm.setArmPosition(Constants.BallArm.PICKUP_POSITION);
            
            // Activate intake
            m_arm.setIntakePower(Constants.BallArm.INTAKE_SPEED);
            
            System.out.println("Arm deployed, activating intake");
        }
        
        // Check if we have successfully collected a ball
        if (m_arm.hasBall()) {
            System.out.println("Ball acquired");
            transitionToState(TrackingState.SECURING);
        }
        
        // If we've been trying for too long without success
        if (m_stateTimer.get() > 2.0) {
            // Try a small forward nudge to improve collection odds
            m_drive.tankDrive(0.1, 0.1);
            
            if (m_stateTimer.get() > 3.0) {
                System.out.println("Collection timeout - no ball detected");
                transitionToState(TrackingState.SECURING);
            }
        }
    }
    
    /**
     * SECURING state - Confirm ball is secure and prepare for return
     */
    private void executeSecuring() {
        // Stop movement
        m_drive.tankDrive(0, 0);
        
        // If ball is detected, hold it securely
        if (m_arm.hasBall()) {
            m_arm.setIntakePower(Constants.BallArm.HOLD_SPEED);
            System.out.println("Ball secured");
        } else {
            // No ball detected, turn off intake
            m_arm.setIntakePower(0);
            System.out.println("No ball detected, collection unsuccessful");
        }
        
        // Return arm to safe position
        m_arm.homeArm();
        
        // Once arm is in motion, move to return state
        if (m_stateTimer.get() > 0.5) {
            transitionToState(TrackingState.RETURNING);
        }
    }
    
    /**
     * RETURNING state - Return to starting position/orientation
     */
    private void executeReturning() {
        // Ensure arm is fully retracted for safe travel
        m_arm.homeArm();
        
        // Use gyro to return to original angle
        double currentAngle = m_drive.getGyroAngle();
        double angleError = m_initialAngle - currentAngle;
        
        // Normalize angle to -180 to 180
        while (angleError > 180) angleError -= 360;
        while (angleError < -180) angleError += 360;
        
        // Calculate turn power
        double turnPower = angleError * 0.01;
        
        // Limit turn power for safety
        turnPower = Math.max(-0.3, Math.min(0.3, turnPower));
        
        // Apply turning power (convert to tank drive)
        m_drive.tankDrive(turnPower, -turnPower);
        
        // Check if we're close to the target angle
        boolean atTargetAngle = Math.abs(angleError) < 5.0;
        
        // Once we're at the target angle, we're done!
        if (atTargetAngle || m_stateTimer.get() > 3.0) {
            m_drive.tankDrive(0, 0);
            System.out.println("Ball tracking sequence complete");
            transitionToState(TrackingState.COMPLETE);
        }
    }
    
    /**
     * Calculate approach speed based on distance
     * 
     * @param distance Distance to target in meters
     * @return Appropriate approach speed
     */
    private double calculateApproachSpeed(double distance) {
        // Base speed proportional to distance
        double speed = Math.min(MAX_DRIVE_SPEED, distance * 0.6);
        
        // Taper speed as we get closer
        if (distance < 0.7) {
            speed = Math.min(speed, 0.3);
        }
        
        if (distance < 0.3) {
            speed = Math.min(speed, 0.2);
        }
        
        // Ensure minimum creep speed 
        speed = Math.max(0.15, speed);
        
        return speed;
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
                timeout = 10.0;
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
                timeout = 5.0;
                break;
            case COMPLETE:
                timeout = Double.POSITIVE_INFINITY;
                break;
            default:
                timeout = 5.0;
                break;
        }
        
        // Check if we've exceeded the timeout
        if (m_stateTimer.get() > timeout) {
            handleTimeout();
        }
        
        // Also check overall command timeout
        if (m_commandTimer.get() > 25.0) {
            System.out.println("Command timeout - ending ball tracking");
            transitionToState(TrackingState.COMPLETE);
        }
    }
    
    /**
     * Handle timeout in current state
     */
    private void handleTimeout() {
        System.out.println("Timeout in state: " + m_state);
        
        switch (m_state) {
            case SCANNING:
                // Scanning timed out - try the other direction
                m_scanDirection *= -1;
                m_scanCount++;
                
                // If we've tried both directions, give up
                if (m_scanCount > 2) {
                    System.out.println("Scanning failed - no balls found");
                    transitionToState(TrackingState.RETURNING);
                } else {
                    System.out.println("Changing scan direction");
                    // Reset timer but stay in scanning state
                    m_stateTimer.reset();
                }
                break;
                
            case TARGETING:
                // Targeting timed out - go back to scanning
                transitionToState(TrackingState.SCANNING);
                break;
                
            case APPROACHING:
                // Approach timed out - try collection anyway
                System.out.println("Approach timed out - attempting collection");
                transitionToState(TrackingState.COLLECTING);
                break;
                
            case COLLECTING:
            case SECURING:
                // Collection timed out - go to return
                System.out.println("Collection timed out - returning to start");
                m_arm.setIntakePower(0);
                m_arm.homeArm();
                transitionToState(TrackingState.RETURNING);
                break;
                
            case RETURNING:
                // Return timed out - just finish
                System.out.println("Return timed out - ending sequence");
                transitionToState(TrackingState.COMPLETE);
                break;
                
            default:
                // For other states, just move to complete
                transitionToState(TrackingState.COMPLETE);
                break;
        }
    }
    
    /**
     * Transition to a new state in our state machine
     * 
     * @param newState The new state to transition to
     */
    private void transitionToState(TrackingState newState) {
        // Log state transition
        System.out.println("State transition: " + m_state + " → " + newState);
        
        // Update state
        m_state = newState;
        
        // Reset state timer
        m_stateTimer.reset();
        m_stateTimer.start();
        
        // Reset any state-specific variables if needed
        if (newState == TrackingState.SCANNING) {
            // Prepare for a new scan
            m_acquiredTarget = false;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Safety first - stop everything
        m_drive.tankDrive(0, 0);
        
        // Ensure arm is in a safe position
        m_arm.homeArm();
        
        // Stop all timers
        m_stateTimer.stop();
        m_commandTimer.stop();
        
        // Status message
        if (interrupted) {
            System.out.println("Ball tracking interrupted");
        } else {
            System.out.println("Ball tracking completed");
            
            // If we have a ball, report success
            if (m_arm.hasBall()) {
                System.out.println("Ball successfully acquired");
            } else {
                System.out.println("No ball acquired");
            }
        }
        
        SmartDashboard.putBoolean("Ball Tracking Active", false);
    }
    
    @Override
    public boolean isFinished() {
        // Only finish when we reach COMPLETE state
        return m_state == TrackingState.COMPLETE;
    }
}
