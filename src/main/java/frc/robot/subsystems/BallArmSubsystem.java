/*
 * ═════════════════════════════════════════════════════════════
 *  ____    _    _    _         _    ____   __  __ 
 * | __ )  / \  | |  | |       / \  |  _ \ |  \/  |
 * |  _ \ / _ \ | |  | |      / _ \ | |_) || |\/| |
 * | |_) / ___ \| |__| |___  / ___ \|  _ < | |  | |
 * |____/_/   \_\____|_____|/_/   \_\_| \_\|_|  |_|
 * ═════════════════════════════════════════════════════════════
 * 
 * TEAM 7221 - REEFSCAPE 2025 - DRAWER SLIDE ARM SYSTEM
 * 
 * This is our precision ball control system using a heavy-duty drawer slide!
 * The system employs a cable-pulley mechanism driven by a NEO motor with 16:1 gearbox
 * for perfect position control, with aluminum C-claws for superior ball grip.
 * 
 * Technical breakdown:
 * - NEO motor with 16:1 gearbox provides excellent torque at arm
 * - Cable-pulley system gives mechanical advantage without slop
 * - C-shaped aluminum gripper with NEO 550 powered wheels
 * - Dual limit switches prevent mechanism damage
 * - Position-based PID control for precise placement
 * 
 * Last updated: March 2025
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * BallArmSubsystem - Controls the drawer slide ball manipulation system
 * 
 * This subsystem manages our cable-driven drawer slide arm with C-shaped 
 * aluminum claws for ball grasping. The system is powered by a NEO motor
 * with 16:1 gearbox for extension and a NEO 550 for the gripper wheels.
 */
public class BallArmSubsystem extends SubsystemBase {
    // ==== HARDWARE COMPONENTS ====
    private final CANSparkMax m_extensionMotor;   // NEO with 16:1 gearbox for drawer slide
    private final CANSparkMax m_gripperMotor;     // NEO 550 for ball intake wheels
    private final RelativeEncoder m_extensionEncoder; // Built-in NEO encoder
    private final SparkPIDController m_pidController; // PID controller for position
    
    // ==== SENSORS ====
    private final DigitalInput m_extendedLimitSwitch;    // Prevents over-extension
    private final DigitalInput m_retractedLimitSwitch;   // Detects full retraction
    private final Ultrasonic m_ballDetector;             // Detects when we have a ball
    
    // ==== STATE VARIABLES ====
    private boolean m_hasBall = false;         // Ball detection status
    private double m_extensionZero = 0.0;      // Zero reference for encoder
    private long m_lastStatusTime = 0;         // For limiting console output
    private int m_stallCounter = 0;            // For detecting jams
    private double m_maxCurrent = 0.0;         // Track highest current draw
    private double m_lastPosition = 0.0;       // For checking movement
    private double m_targetPosition = 0.0;     // Desired arm position
    private boolean m_statusBlink = false;     // For dashboard indicator

    /**
     * Creates the drawer slide ball arm subsystem
     */
    public BallArmSubsystem() {
        System.out.println("\n" +
            "╔════════════════════════════════════════════════╗\n" +
            "║     DRAWER SLIDE ARM SUBSYSTEM INITIALIZING    ║\n" +
            "║       NEO WITH 16:1 GEARBOX ACTIVATED          ║\n" +
            "║       CABLE-DRIVE MECHANISM COMING ONLINE      ║\n" +
            "╚════════════════════════════════════════════════╝");
        
        // Initialize extension motor - NEO with 16:1 gearbox
        m_extensionMotor = new CANSparkMax(Constants.BallArm.EXTENSION_MOTOR_ID, MotorType.kBrushless);
        m_extensionEncoder = m_extensionMotor.getEncoder();
        m_pidController = m_extensionMotor.getPIDController();
        
        // Initialize gripper motor - NEO 550
        m_gripperMotor = new CANSparkMax(Constants.BallArm.GRIPPER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure motor settings
        configureMotors();
        
        // Initialize sensors
        m_extendedLimitSwitch = new DigitalInput(Constants.Electrical.BALL_ARM_EXTENDED_LIMIT_PORT);
        m_retractedLimitSwitch = new DigitalInput(Constants.Electrical.BALL_ARM_RETRACTED_LIMIT_PORT);
        
        // Initialize ultrasonic sensor for ball detection
        m_ballDetector = new Ultrasonic(
            Constants.Electrical.BALL_DETECTOR_PING_PORT,
            Constants.Electrical.BALL_DETECTOR_ECHO_PORT
        );
        
        // Enable continuous ultrasonic readings
        Ultrasonic.setAutomaticMode(true);
        
        // Reset encoder to establish zero position
        resetArmEncoder();
        
        // Show ASCII art to visualize the mechanism
        System.out.println("   ╔═══════════════════════════════╗");
        System.out.println("   ║   DRAWER SLIDE ARM ACTIVATED  ║");
        System.out.println("   ╚═════════════╦═════════════════╝");
        System.out.println("                 ║");
        System.out.println("        ┌────────┴─────────┐");
        System.out.println("        │    NEO 16:1      │");
        System.out.println("        └────────┬─────────┘");
        System.out.println("                 ║");
        System.out.println("        ┌────────┴─────────┐");
        System.out.println("        │   DRAWER SLIDE   │");
        System.out.println("        └────────┬─────────┘");
        System.out.println("                 ║");
        System.out.println("           ┌─────┴─────┐");
        System.out.println("           C           C");
        System.out.println("           │     O     │");
        System.out.println("           └───────────┘");
    }
    
    /**
     * Configure motor controllers for optimal performance
     * Settings tuned specifically for drawer slide application
     */
    private void configureMotors() {
        // Configure extension motor (NEO with 16:1 gearbox)
        m_extensionMotor.restoreFactoryDefaults();
        m_extensionMotor.setInverted(Constants.BallArm.EXTENSION_MOTOR_INVERTED);
        m_extensionMotor.setIdleMode(IdleMode.kBrake); // Brake mode essential for position holding
        m_extensionMotor.setSmartCurrentLimit(30); // 30A limit protects the motor
        m_extensionMotor.enableVoltageCompensation(11.0);
        m_extensionMotor.setOpenLoopRampRate(0.1); // 100ms ramp for smooth acceleration
        m_extensionMotor.setClosedLoopRampRate(0.1);
        
        // Configure PID controller for position control
        m_pidController.setP(Constants.BallArm.POSITION_KP);
        m_pidController.setI(Constants.BallArm.POSITION_KI);
        m_pidController.setD(Constants.BallArm.POSITION_KD);
        m_pidController.setFF(Constants.BallArm.POSITION_KF);
        m_pidController.setOutputRange(-Constants.BallArm.MAX_SPEED, Constants.BallArm.MAX_SPEED);
        
        // Configure gripper motor (NEO 550)
        m_gripperMotor.restoreFactoryDefaults();
        m_gripperMotor.setInverted(Constants.BallArm.GRIPPER_MOTOR_INVERTED);
        m_gripperMotor.setIdleMode(IdleMode.kBrake);
        m_gripperMotor.setSmartCurrentLimit(20); // Lower limit for NEO 550
        m_gripperMotor.enableVoltageCompensation(11.0);
        
        // Save configuration to flash memory
        m_extensionMotor.burnFlash();
        m_gripperMotor.burnFlash();
        
        System.out.println(">> Motors configured: Extension (NEO 16:1), Gripper (NEO 550)");
        System.out.printf(">> PID values: P=%.4f, I=%.4f, D=%.4f, F=%.4f\n", 
                          Constants.BallArm.POSITION_KP, Constants.BallArm.POSITION_KI, 
                          Constants.BallArm.POSITION_KD, Constants.BallArm.POSITION_KF);
    }
    
    /**
     * Reset arm encoder to establish zero position
     */
    public void resetArmEncoder() {
        m_extensionEncoder.setPosition(0);
        m_extensionZero = 0;
        m_targetPosition = 0;
        System.out.println(">> ARM ENCODER ZEROED: Position tracking calibrated");
    }
    
    /**
     * Get the arm position in encoder units relative to zero point
     * @return Current position in rotations
     */
    public double getArmPosition() {
        return m_extensionEncoder.getPosition() - m_extensionZero;
    }
    
    /**
     * Get current draw from the arm extension motor
     * @return Current in amps
     */
    public double getArmCurrent() {
        double current = m_extensionMotor.getOutputCurrent();
        
        // Track maximum current for diagnostic purposes
        if (current > m_maxCurrent) {
            m_maxCurrent = current;
        }
        
        return current;
    }
    
    /**
     * Move the drawer slide arm using direct speed control
     * @param speed Speed to move (-1.0 to 1.0), positive = extend, negative = retract
     */
    public void moveArm(double speed) {
        // Safety checks - verify limit switches before moving
        // Note: switches return TRUE when NOT pressed
        boolean canExtend = m_extendedLimitSwitch.get();
        boolean canRetract = m_retractedLimitSwitch.get();
        
        // Only allow movement if not at a limit
        if ((speed > 0 && canExtend) || (speed < 0 && canRetract)) {
            // Apply motor power with cable-drive compensation
            // Cable systems require extra power to overcome initial tension
            double compensatedSpeed = speed;
            if (Math.abs(speed) < 0.15 && Math.abs(speed) > 0.05) {
                // Add extra power to overcome cable static friction
                compensatedSpeed = speed * 1.2;
            }
            
            // Apply the motor power
            m_extensionMotor.set(compensatedSpeed);
            
            // Check for stalls by monitoring position changes
            double currentPosition = getArmPosition();
            if (Math.abs(speed) > 0.1 && Math.abs(currentPosition - m_lastPosition) < 0.01) {
                m_stallCounter++;
                
                // If stalled for several cycles, log a warning
                if (m_stallCounter > 20) {
                    System.out.println(">> WARNING: Potential mechanism jam detected");
                    System.out.printf("   Position: %.2f, Current: %.1fA\n", 
                                     currentPosition, getArmCurrent());
                }
            } else {
                // Reset stall counter if moving normally
                m_stallCounter = 0;
            }
            
            // Update position tracking
            m_lastPosition = currentPosition;
        } else {
            // At a limit - stop motor
            m_extensionMotor.set(0);
            
            // Log limit events (but don't spam logs)
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 1000) {
                if (speed > 0 && !canExtend) {
                    System.out.println(">> ARM: At maximum extension limit");
                } else if (speed < 0 && !canRetract) {
                    System.out.println(">> ARM: At minimum retraction limit");
                }
                m_lastStatusTime = now;
            }
        }
    }
    
    /**
     * Set the gripper wheels to intake or release balls
     * @param speed Speed to run the gripper (-1.0 to 1.0)
     */
    public void setGripper(double speed) {
        // Apply wheel speed with deadband for small values
        if (Math.abs(speed) < 0.05) {
            m_gripperMotor.set(0);
        } else {
            m_gripperMotor.set(speed);
        }
        
        // Log significant state changes
        if (Math.abs(speed) > 0.3) {
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 500) {
                if (speed > 0) {
                    System.out.println(">> GRIPPER: Intaking at " + (speed * 100) + "% power");
                } else {
                    System.out.println(">> GRIPPER: Ejecting at " + (Math.abs(speed) * 100) + "% power");
                }
                m_lastStatusTime = now;
            }
        }
    }
    
    /**
     * Check if a ball is detected in the gripper
     * @return true if ball is detected
     */
    public boolean hasBall() {
        // Get distance reading from ultrasonic sensor
        double rangeInches = m_ballDetector.getRangeInches();
        
        // Ball detected if distance is below threshold
        boolean ballDetected = rangeInches < Constants.BallArm.DETECTION_THRESHOLD_INCHES;
        
        // Announce new ball detection without spamming logs
        if (ballDetected && !m_hasBall) {
            System.out.println("\n>> BALL ACQUIRED! Distance: " + 
                             String.format("%.1f", rangeInches) + " inches");
            System.out.println(">> Gripper secure - Ready for transport\n");
        } else if (!ballDetected && m_hasBall) {
            System.out.println("\n>> BALL RELEASED! Gripper now empty\n");
        }
        
        // Update state tracking
        m_hasBall = ballDetected;
        
        return ballDetected;
    }
    
    /**
     * Set arm to a specific position using PID control
     * @param targetPosition Target position in encoder units
     */
    public void setArmPosition(double targetPosition) {
        // Enforce position limits for safety
        if (targetPosition < Constants.BallArm.MIN_POSITION) {
            targetPosition = Constants.BallArm.MIN_POSITION;
            System.out.println(">> Position limited to minimum safe extension");
        } else if (targetPosition > Constants.BallArm.MAX_POSITION) {
            targetPosition = Constants.BallArm.MAX_POSITION;
            System.out.println(">> Position limited to maximum safe extension");
        }
        
        // Update target tracking
        m_targetPosition = targetPosition;
        
        // Apply position setpoint to PID controller
        m_pidController.setReference(targetPosition + m_extensionZero, 
                                   CANSparkMax.ControlType.kPosition);
        
        // Log significant position changes
        double currentPosition = getArmPosition();
        double error = Math.abs(targetPosition - currentPosition);
        
        if (error > 0.5) {
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 500) {
                System.out.printf(">> ARM: Moving to %.2f (current: %.2f, error: %.2f)\n", 
                                targetPosition, currentPosition, error);
                m_lastStatusTime = now;
            }
        }
    }
    
    /**
     * Move arm to fully retracted home position
     */
    public void homeArm() {
        System.out.println(">> COMMAND: Retracting arm to home position");
        setArmPosition(Constants.BallArm.HOME_POSITION);
    }
    
    /**
     * Move arm to pickup position for floor ball acquisition
     */
    public void pickupPosition() {
        System.out.println("\n>> COMMAND: Extending arm to floor pickup position");
        System.out.println(">> Ball acquisition sequence initiated\n");
        setArmPosition(Constants.BallArm.PICKUP_POSITION);
    }
    
    /**
     * Move arm to scoring position at full extension
     */
    public void scorePosition() {
        System.out.println("\n>> COMMAND: Extending arm to scoring position");
        System.out.println(">> Preparing for ball release\n");
        setArmPosition(Constants.BallArm.SCORE_POSITION);
    }
    
    /**
     * Automatically intake a ball at optimal speed
     */
    public void intakeBall() {
        // Run intake at full power if no ball detected
        if (!hasBall()) {
            setGripper(Constants.BallArm.GRIPPER_INTAKE_SPEED);
        } else {
            // Reduce to holding speed once ball is secured
            setGripper(Constants.BallArm.GRIPPER_HOLD_SPEED);
        }
    }
    
    /**
     * Release ball at high speed for scoring
     */
    public void releaseBall() {
        System.out.println(">> LAUNCHING BALL!");
        setGripper(Constants.BallArm.GRIPPER_RELEASE_SPEED);
    }
    
    /**
     * Emergency stop for the arm system
     */
    public void emergencyStop() {
        m_extensionMotor.set(0);
        m_gripperMotor.set(0);
        System.out.println("!!! ARM EMERGENCY STOP ACTIVATED !!!");
    }
    
    @Override
    public void periodic() {
        // Track ball state and report to dashboard
        boolean currentBallState = hasBall();
        SmartDashboard.putBoolean("Has Ball", currentBallState);
        
        // Blink status indicator for dashboard
        long now = System.currentTimeMillis();
        if (now - m_lastStatusTime > 250) {
            m_statusBlink = !m_statusBlink;
            m_lastStatusTime = now;
        }
        
        // Report arm position and status to dashboard
        double currentPosition = getArmPosition();
        SmartDashboard.putNumber("Arm Position", currentPosition);
        SmartDashboard.putNumber("Arm Target", m_targetPosition);
        SmartDashboard.putNumber("Arm Current", getArmCurrent());
        SmartDashboard.putBoolean("At Extension Limit", !m_extendedLimitSwitch.get());
        SmartDashboard.putBoolean("At Retraction Limit", !m_retractedLimitSwitch.get());
        SmartDashboard.putBoolean("Status Indicator", m_statusBlink);
        
        // Check for excessive current draw
        double current = getArmCurrent();
        if (current > Constants.BallArm.STALL_CURRENT_THRESHOLD) {
            // Log warning if sustained high current
            if (now - m_lastStatusTime > 1000) {
                System.out.println(">> WARNING: High arm current: " + 
                                 String.format("%.1f", current) + "A");
            }
        }
    }
    
    /**
     * Get the extension motor for external access
     * @return The extension motor controller
     */
    public CANSparkMax getExtensionMotor() {
        return m_extensionMotor;
    }
    
    /**
     * Get the gripper motor for external access
     * @return The gripper motor controller
     */
    public CANSparkMax getGripperMotor() {
        return m_gripperMotor;
    }
}
