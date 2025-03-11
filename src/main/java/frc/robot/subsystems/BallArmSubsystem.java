// src/main/java/frc/robot/subsystems/BallArmSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * ┌───────────────────────────────────────────────────────┐
 * │           BALL ARM SUBSYSTEM - TEAM 7221              │
 * │         REEFSCAPE COMPETITION ROBOT 2025              │
 * └───────────────────────────────────────────────────────┘
 * 
 * This subsystem controls the robot's arm mechanism for manipulating
 * game pieces (CORAL and ALGAE) during the 2025 Reefscape game.
 * 
 * The arm uses:
 * - Extension motor: Controls the drawer slide extension
 * - Gripper motor: Controls intake wheels for grabbing game pieces
 * - Limit switches: Detect arm extension limits
 * - Ultrasonic sensor: Detects ball presence for automated pickup
 * 
 * NOTE: This is a modified version that works without the REV Robotics
 * library. For full functionality, install the REV vendor dependency
 * through the WPILib VSCode extension.
 */
public class BallArmSubsystem extends SubsystemBase {
    
    // ===== HARDWARE COMPONENTS =====
    
    // Motors - Using generic MotorController interface for flexibility
    private MotorController armExtensionMotor;
    private MotorController gripperMotor;
    
    // Sensors
    private final DigitalInput upperLimitSwitch;
    private final DigitalInput lowerLimitSwitch;
    private final DigitalInput ballDetector;
    
    // ===== STATE TRACKING =====
    private boolean hasBall = false;
    private boolean isMoving = false;
    private double targetPosition = 0.0;
    private double currentPosition = 0.0;
    
    /**
     * Creates a new BallArmSubsystem.
     * This constructor initializes all hardware needed for ball manipulation.
     */
    public BallArmSubsystem() {
        System.out.println("╔════════════════════════════════════════════════╗");
        System.out.println("║       INITIALIZING BALL ARM SUBSYSTEM          ║");
        System.out.println("╚════════════════════════════════════════════════╝");
        
        // Initialize motors using PWM as fallback
        // NOTE: These should be CANSparkMax controllers when the REV library is available
        try {
            // Using PWM as fallback - not ideal but allows basic functionality
            armExtensionMotor = new PWMSparkMax(Constants.BallArm.EXTENSION_MOTOR_ID);
            gripperMotor = new PWMSparkMax(Constants.BallArm.GRIPPER_MOTOR_ID);
            
            // Configure motor directions
            armExtensionMotor.setInverted(Constants.BallArm.EXTENSION_MOTOR_INVERTED);
            gripperMotor.setInverted(Constants.BallArm.GRIPPER_MOTOR_INVERTED);
            
            System.out.println(">> Motors initialized in PWM mode (limited functionality)");
            System.out.println(">> Install REV Robotics vendor library for full functionality");
        } catch (Exception e) {
            System.err.println("!!! ERROR INITIALIZING MOTORS: " + e.getMessage());
            System.err.println("!!! Ball arm will not be functional");
        }
        
        // Initialize limit switches and ball detector
        upperLimitSwitch = new DigitalInput(Constants.Electrical.BALL_ARM_EXTENDED_LIMIT_PORT);
        lowerLimitSwitch = new DigitalInput(Constants.Electrical.BALL_ARM_RETRACTED_LIMIT_PORT);
        ballDetector = new DigitalInput(Constants.Electrical.BALL_DETECTOR_PING_PORT);
        
        System.out.println(">> Arm sensors initialized");
        
        // Set initial state
        targetPosition = Constants.BallArm.HOME_POSITION;
        
        SmartDashboard.putString("Ball Arm Status", "Initialized");
        System.out.println(">> Ball arm subsystem initialization complete");
    }
    
    /**
     * This method is called periodically by the CommandScheduler.
     * Updates sensor readings, checks safety conditions, and logs information.
     */
    @Override
    public void periodic() {
        // Update ball detection status (switches are typically active-low)
        hasBall = !ballDetector.get();
        
        // Check limit switches for safety
        checkLimitSwitches();
        
        // Update dashboard with current state
        updateDashboard();
        
        // Simple position tracking (simulated without encoders)
        if (isMoving) {
            // Estimate position change based on motor output
            if (armExtensionMotor != null) {
                double motorOutput = armExtensionMotor.get();
                if (Math.abs(motorOutput) > 0.05) {
                    currentPosition += motorOutput * 0.05; // Simple simulation
                    currentPosition = Math.max(Constants.BallArm.MIN_POSITION, 
                                        Math.min(currentPosition, Constants.BallArm.MAX_POSITION));
                }
            }
        }
    }
    
    /**
     * Moves the arm using direct speed control.
     * Positive values extend, negative values retract.
     * 
     * @param speed The speed to move the arm (-1.0 to 1.0)
     */
    public void moveArm(double speed) {
        // Apply safety limits
        speed = Math.max(-Constants.BallArm.MAX_SPEED, Math.min(speed, Constants.BallArm.MAX_SPEED));
        
        // Check limit switches
        if ((speed > 0 && isAtUpperLimit()) || (speed < 0 && isAtLowerLimit())) {
            // Don't move past limits
            stopArm();
            return;
        }
        
        // Apply speed to motor
        if (armExtensionMotor != null) {
            armExtensionMotor.set(speed);
            isMoving = Math.abs(speed) > 0.05;
            
            if (isMoving) {
                SmartDashboard.putString("Ball Arm Status", "Moving: " + 
                                        (speed > 0 ? "Extending" : "Retracting"));
            }
        }
    }
    
    /**
     * Stops the arm movement immediately.
     */
    public void stopArm() {
        if (armExtensionMotor != null) {
            armExtensionMotor.set(0);
            isMoving = false;
            SmartDashboard.putString("Ball Arm Status", "Stopped");
        }
    }
    
    /**
     * Controls the gripper wheels to grab or release balls.
     * 
     * @param speed The speed for the gripper (-1.0 to 1.0)
     *              Positive values intake, negative values expel
     */
    public void setGripper(double speed) {
        // Apply speed limits
        speed = Math.max(-1.0, Math.min(speed, 1.0));
        
        // Apply speed to gripper motor
        if (gripperMotor != null) {
            gripperMotor.set(speed);
            
            if (Math.abs(speed) > 0.05) {
                SmartDashboard.putString("Intake Status", 
                                       speed > 0 ? "Intaking" : "Outtaking");
            } else {
                SmartDashboard.putString("Intake Status", "Stopped");
            }
        }
    }
    
    /**
     * Move arm to home (stowed) position.
     */
    public void homeArm() {
        // Move arm to home position
        // Without encoders, we just retract until limit switch
        if (isAtLowerLimit()) {
            stopArm();
        } else {
            moveArm(-Constants.BallArm.MAX_SPEED * 0.7); // 70% speed for gentler movement
        }
        SmartDashboard.putString("Ball Arm Status", "Homing");
    }
    
    /**
     * Move arm to pickup position.
     */
    public void pickupPosition() {
        // Move to pickup position
        setArmPosition(Constants.BallArm.PICKUP_POSITION);
        SmartDashboard.putString("Ball Arm Status", "Moving to Pickup Position");
    }
    
    /**
     * Move arm to scoring position.
     */
    public void scorePosition() {
        // Move to scoring position
        setArmPosition(Constants.BallArm.SCORE_POSITION);
        SmartDashboard.putString("Ball Arm Status", "Moving to Score Position");
    }
    
    /**
     * Release ball with high speed outtake.
     */
    public void releaseBall() {
        setGripper(-Constants.BallArm.GRIPPER_RELEASE_SPEED);
        SmartDashboard.putString("Intake Status", "Releasing Ball");
    }
    
    /**
     * Set target arm position (simplified without encoder feedback).
     * Uses a basic state machine approach.
     * 
     * @param position The target position in arbitrary units
     */
    public void setArmPosition(double position) {
        // Clamp the position to safe limits
        position = Math.min(Math.max(position, Constants.BallArm.MIN_POSITION), 
                           Constants.BallArm.MAX_POSITION);
        
        // Store the target
        targetPosition = position;
        
        // Simple open-loop control based on current estimated position
        if (Math.abs(currentPosition - targetPosition) > 0.5) {
            if (currentPosition < targetPosition) {
                // Need to extend
                if (!isAtUpperLimit()) {
                    moveArm(Constants.BallArm.MAX_SPEED * 0.8);
                }
            } else {
                // Need to retract
                if (!isAtLowerLimit()) {
                    moveArm(-Constants.BallArm.MAX_SPEED * 0.8);
                }
            }
        } else {
            // Close enough, stop
            stopArm();
        }
    }
    
    /**
     * Checks if the arm is at its upper limit.
     * 
     * @return true if the upper limit switch is activated
     */
    public boolean isAtUpperLimit() {
        return !upperLimitSwitch.get(); // Switches are typically active-low
    }
    
    /**
     * Checks if the arm is at its lower limit.
     * 
     * @return true if the lower limit switch is activated
     */
    public boolean isAtLowerLimit() {
        return !lowerLimitSwitch.get(); // Switches are typically active-low
    }
    
    /**
     * Checks if the robot currently has a ball.
     * 
     * @return true if a ball is detected
     */
    public boolean hasBall() {
        return hasBall;
    }
    
    /**
     * Gets the current position of the arm.
     * This is an estimation without encoders.
     * 
     * @return The current position in arbitrary units
     */
    public double getArmPosition() {
        return currentPosition;
    }
    
    /**
     * Checks limit switches and stops the motor if limits are reached.
     * This is a safety feature to prevent mechanical damage.
     */
    private void checkLimitSwitches() {
        if (armExtensionMotor != null && isMoving) {
            // If moving up and hit upper limit, stop
            if (armExtensionMotor.get() > 0 && isAtUpperLimit()) {
                stopArm();
                currentPosition = Constants.BallArm.MAX_POSITION;
            }
            
            // If moving down and hit lower limit, stop
            if (armExtensionMotor.get() < 0 && isAtLowerLimit()) {
                stopArm();
                currentPosition = Constants.BallArm.MIN_POSITION;
            }
        }
    }
    
    /**
     * Updates the SmartDashboard with current subsystem information.
     */
    private void updateDashboard() {
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Has Ball", hasBall());
        SmartDashboard.putBoolean("At Upper Limit", isAtUpperLimit());
        SmartDashboard.putBoolean("At Lower Limit", isAtLowerLimit());
        
        // Motor temperatures not available in PWM mode
        // SmartDashboard.putNumber("Arm Motor Temperature", armExtensionMotor.getMotorTemperature());
    }
    
    /**
     * Prepares the arm for the autonomous period.
     * Sets the arm to a known starting position.
     */
    public void prepareForAuto() {
        // Move to the starting position for autonomous
        homeArm();
    }
    
    /**
     * Prepares the arm for teleop control.
     */
    public void prepareForTeleop() {
        // Any specific setup for teleop
        stopArm();
        setGripper(0);
    }
}
