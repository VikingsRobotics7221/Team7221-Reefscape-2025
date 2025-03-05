/*
 * ===============================================================
 *  ____    _    _     _          _    ____   __  __ 
 * | __ )  / \  | |   | |        / \  |  _ \ |  \/  |
 * |  _ \ / _ \ | |   | |       / _ \ | |_) || |\/| |
 * | |_) / ___ \| |___| |___   / ___ \|  _ < | |  | |
 * |____/_/   \_\_____|_____| /_/   \_\_| \_\|_|  |_|
 *                                                   
 * ===============================================================
 * 
 * TEAM 7221 - REEFSCAPE 2025 - DRAWER SLIDE ARM SYSTEM
 * 
 * This is our EPIC ball control system using that heavy-duty drawer slide!
 * We're using a cable-pulley system to drive it with our 16:1 NEO.
 * Those C-claws are gonna grab balls like NOTHING ELSE at competition!
 * 
 * The whole system uses one drawer slide powered by a 16:1 NEO with a cable 
 * drive, and a NEO 550 for the gripper wheels. Simple but POWERFUL design!
 * 
 * Coded by paysean (spent 3 days getting this right!)
 * Last updated: March 2025
 */

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * BallArmSubsystem - Our secret weapon for Reefscape 2025!
 * 
 * Controls our cable-driven drawer slide arm with C-shaped aluminum claws
 * for grabbing game pieces. Using a NEO with 16:1 gearing for extension
 * and a NEO 550 for the gripper wheels.
 * 
 * This thing is going to DOMINATE the competition with its reach and speed!
 * Worked on this for days getting the wire routing right and PID tuning perfect.
 */
public class BallArmSubsystem extends SubsystemBase {
    // ===== MOTOR CONTROLLERS - THE MUSCLES =====
    private final SparkMax m_extensionMotor; // NEO with 16:1 gearing for drawer extension
    private final SparkMax m_gripperMotor; // NEO 550 for ball intake wheels
    
    // ===== SENSORS - THE EYES AND EARS =====
    private final DigitalInput m_extendedLimitSwitch; // Prevents over-extension
    private final DigitalInput m_retractedLimitSwitch; // Detects full retraction
    private final Ultrasonic m_ballDetector; // Detects when we have a ball
    
    // ===== STATE VARIABLES - THE BRAINS =====
    private boolean m_hasBall = false; // Ball detection status
    private double m_extensionZero = 0.0; // Zero reference for encoder
    private long m_lastStatusTime = 0; // For limiting console output
    private boolean m_statusBlink = false; // For dashboard indicator
    private int m_stallCounter = 0; // For detecting jams
    
    // ===== PERFORMANCE TRACKING =====
    private double m_maxCurrent = 0.0; // Track highest current draw
    private double m_lastPosition = 0.0; // For checking if we're actually moving

    /**
     * Creates the drawer slide ball arm subsystem - THE BALL GRABBING BEAST!
     */
    public BallArmSubsystem() {
        System.out.println("\n" +
                           "======================================================\n" +
                           ">> INITIALIZING DRAWER SLIDE BALL ARM SUBSYSTEM!!\n" +
                           ">> THE SECRET WEAPON OF TEAM 7221 IS COMING ONLINE!!!!\n" +
                           ">> PREPARE FOR MAXIMUM EXTENSION AND BALL ACQUISITION!!\n" +
                           "======================================================");
        
        // Initialize motors - THE POWER PLANTS
        m_extensionMotor = new SparkMax(Constants.BALL_ARM_EXTENSION_MOTOR_ID, MotorType.kBrushless);
        m_gripperMotor = new SparkMax(Constants.BALL_GRIPPER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure motor settings - OPTIMIZATION TIME
        configureSparkMAX(m_extensionMotor, Constants.BALL_ARM_EXTENSION_MOTOR_INVERTED);
        configureSparkMAX(m_gripperMotor, Constants.BALL_GRIPPER_MOTOR_INVERTED);
        
        // Initialize sensors - THE DETECTION SYSTEMS
        m_extendedLimitSwitch = new DigitalInput(Constants.BALL_ARM_EXTENDED_LIMIT_PORT);
        m_retractedLimitSwitch = new DigitalInput(Constants.BALL_ARM_RETRACTED_LIMIT_PORT);
        
        // Initialize ultrasonic sensor - THE BALL DETECTOR
        m_ballDetector = new Ultrasonic(
            Constants.BALL_DETECTOR_PING_PORT,
            Constants.BALL_DETECTOR_ECHO_PORT
        );
        
        // Enable continuous ultrasonic readings
        Ultrasonic.setAutomaticMode(true);
        
        // Reset encoder to establish zero position
        resetArmEncoder();
        
        // Show a cool ASCII art to celebrate successful initialization
        System.out.println("     /\\_/\\  "); 
        System.out.println("    ( >w< ) DRAWER SLIDE ARM ACTIVATED!");
        System.out.println("    /|___|\\  READY TO GRAB BALLS AND DOMINATE!");
        System.out.println("   / |   | \\ ");
        System.out.println("  *  |___|  *");
    }
    
    /**
     * Configure motor controllers with ULTIMATE settings for performance
     * Seriously, I spent hours tuning these settings for perfect response
     * 
     * @param motor Motor to configure (NEO or NEO 550)
     * @param inverted Whether to invert direction
     */
    private void configureSparkMAX(SparkMax motor, boolean inverted) {
        // Create configuration for this motor
        SparkMaxConfig config = new SparkMaxConfig();
        
        // BRAKE MODE IS ESSENTIAL for drawer slides!
        // Without this they just coast and it's sloppy AF
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        
        // Apply configuration to motor controller
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Set current limit to protect the motor (these NEOs can pull SERIOUS amps)
        try {
            motor.setSmartCurrentLimit(30); // 30A limit is perfect for our application
            System.out.println(">> Current limit set to 30A for motor protection");
        } catch (Exception e) {
            System.out.println(">> WARNING: Couldn't set current limit. DANGER: Watch your motors!");
            e.printStackTrace();
        }
    }
    
    /**
     * Reset arm encoder to establish zero position
     * This is CRITICAL for accurate positioning
     */
    public void resetArmEncoder() {
        m_extensionZero = m_extensionMotor.getEncoder().getPosition();
        System.out.println(">> ARM ENCODER ZEROED! TRACKING SYSTEM CALIBRATED!");
    }
    
    /**
     * Get the arm position in encoder units relative to our zero point
     * 
     * @return Current position relative to zero
     */
    public double getArmPosition() {
        return m_extensionMotor.getEncoder().getPosition() - m_extensionZero;
    }
    
    /**
     * Move the drawer slide arm - THE CORE FUNCTION
     * 
     * @param speed Speed to move (-1.0 to 1.0), positive = extend, negative = retract
     */
    public void moveArm(double speed) {
        // SAFETY FIRST! Check limit switches before moving
        // Remember: switches return TRUE when NOT pressed (weird, I know)
        boolean canExtend = m_extendedLimitSwitch.get();
        boolean canRetract = m_retractedLimitSwitch.get();
        
        // Only allow movement if not at a limit
        if ((speed > 0 && canExtend) || (speed < 0 && canRetract)) {
            // Apply motor power - MOVE THAT ARM!
            m_extensionMotor.set(speed);
            
            // Check if we're moving properly - this is my anti-jam code
            double currentPosition = getArmPosition();
            if (Math.abs(speed) > 0.1 && Math.abs(currentPosition - m_lastPosition) < 0.01) {
                m_stallCounter++;
                
                // If we've been stalled for a while, something's wrong
                if (m_stallCounter > 25) {
                    System.out.println(">> WARNING: ARM MIGHT BE JAMMED! Movement detected: " + 
                                      (currentPosition - m_lastPosition));
                }
            } else {
                m_stallCounter = 0; // Reset counter if we're moving
            }
            m_lastPosition = currentPosition;
            
        } else {
            // Stop motor if limit reached - SAFETY NEVER SLEEPS
            m_extensionMotor.set(0);
            
            // Log limit events, but don't spam the console
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 1000) {
                if (speed > 0 && !canExtend) {
                    System.out.println(">> ARM: REACHED FULL EXTENSION LIMIT!");
                } else if (speed < 0 && !canRetract) {
                    System.out.println(">> ARM: REACHED FULL RETRACTION LIMIT!");
                }
                m_lastStatusTime = now;
            }
        }
    }
    
    /**
     * Set the gripper wheels to intake or release balls
     * 
     * @param speed Speed to run the gripper (-1.0 to 1.0)
     */
    public void setGripper(double speed) {
        // Spin those intake wheels! GET THAT BALL!
        m_gripperMotor.set(speed);
        
        // Debug info for important state changes
        if (speed > 0.1) {
            System.out.println(">> BALL INTAKE MODE ACTIVATED: " + speed);
        } else if (speed < -0.1) {
            System.out.println(">> BALL LAUNCH MODE ACTIVATED: " + speed);
        }
    }
    
    /**
     * Check if a ball is detected in the gripper
     * Uses ultrasonic sensor to detect presence
     * 
     * @return true if ball is detected
     */
    public boolean hasBall() {
        // Get distance reading from ultrasonic sensor
        double rangeInches = m_ballDetector.getRangeInches();
        
        // Ball detected if distance is below threshold
        boolean ballDetected = rangeInches < Constants.BALL_DETECTION_THRESHOLD_INCHES;
        
        // Announce new ball detection - CELEBRATE OUR VICTORY!
        if (ballDetected && !m_hasBall) {
            System.out.println("");
            System.out.println(">> BALL ACQUIRED! LOCKED AND LOADED!");
            System.out.println("   Distance: " + rangeInches + " inches");
            System.out.println("   MISSION ACCOMPLISHED!");
            System.out.println("");
        }
        
        return ballDetected;
    }
    
    /**
     * Set arm to a specific position using PID control
     * This is where the REAL magic happens - smooth positioning
     * 
     * @param targetPosition Target position in encoder units
     */
    public void setArmPosition(double targetPosition) {
        // Enforce position limits - SAFETY BOUNDARIES
        if (targetPosition < Constants.BALL_ARM_MIN_POSITION) {
            targetPosition = Constants.BALL_ARM_MIN_POSITION;
            System.out.println(">> ARM POSITION LIMITED TO MINIMUM EXTENSION!");
        } else if (targetPosition > Constants.BALL_ARM_MAX_POSITION) {
            targetPosition = Constants.BALL_ARM_MAX_POSITION;
            System.out.println(">> ARM POSITION LIMITED TO MAXIMUM EXTENSION!");
        }
        
        // Calculate position error
        double currentPosition = getArmPosition();
        double error = targetPosition - currentPosition;
        
        // Calculate motor output with P control
        // I spent HOURS tuning this P constant for smooth motion
        double output = Constants.BALL_ARM_KP * error;
        
        // Add a little feed-forward to overcome static friction
        // This makes a HUGE difference in smoothness
        if (Math.abs(error) > 0.1) {
            output += Math.signum(error) * 0.05;
        }
        
        // Limit output to safe speed range
        output = Math.max(-Constants.BALL_ARM_MAX_SPEED, 
                         Math.min(Constants.BALL_ARM_MAX_SPEED, output));
        
        // Apply calculated motor output - MOVE THAT ARM!
        moveArm(output);
        
        // Log significant arm movement (but not too often)
        if (Math.abs(error) > 0.5) {
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 500) {
                System.out.printf(">> ARM: Moving to %.2f (current: %.2f, error: %.2f)\n", 
                                 targetPosition, currentPosition, error);
                m_lastStatusTime = now;
            }
        }
        
        // Reset stall counter when near target position
        if (Math.abs(error) < 0.2) {
            m_stallCounter = 0;
        }
    }
    
    /**
     * Move arm to fully retracted home position
     * This is our safe travel configuration
     */
    public void homeArm() {
        System.out.println(">> RETRACTING ARM TO HOME POSITION!");
        setArmPosition(Constants.BALL_ARM_HOME_POSITION);
    }
    
    /**
     * Move arm to pickup position (partial extension)
     * Perfect for grabbing balls off the floor
     */
    public void pickupPosition() {
        System.out.println("");
        System.out.println(">> EXTENDING ARM TO PICKUP POSITION!");
        System.out.println(">> BALL ACQUISITION MODE ENGAGED!");
        System.out.println("");
        setArmPosition(Constants.BALL_ARM_PICKUP_POSITION);
    }
    
    /**
     * Move arm to scoring position (full extension)
     * Maximum reach for scoring those sweet, sweet points
     */
    public void scorePosition() {
        System.out.println("");
        System.out.println(">> EXTENDING ARM TO MAXIMUM SCORING POSITION!");
        System.out.println(">> PREPARE TO SCORE SOME POINTS!");
        System.out.println("");
        setArmPosition(Constants.BALL_ARM_SCORE_POSITION);
    }
    
    /**
     * Automatically intake a ball - convenience method
     * Handles gripper speed based on ball detection
     */
    public void intakeBall() {
        // Run intake if no ball detected
        if (!m_hasBall) {
            setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
            System.out.println(">> BALL INTAKE ACTIVATED! POWER: " + 
                              Constants.BALL_GRIPPER_INTAKE_SPEED * 100 + "%");
        } else {
            // Hold ball securely if already acquired
            setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
            System.out.println(">> HOLDING BALL SECURELY!");
        }
    }
    
    /**
     * Release ball at high speed - LAUNCH MODE!
     * For when you need to score from a distance
     */
    public void releaseBall() {
        System.out.println("");
        System.out.println(">> RELEASING BALL AT MAXIMUM VELOCITY!");
        System.out.println(">> LAUNCH SEQUENCE INITIATED!");
        System.out.println("");
        setGripper(Constants.BALL_GRIPPER_RELEASE_SPEED);
    }
    
    /**
     * Track the system performance metrics
     * This is super useful for debugging and optimization
     */
    private void trackPerformanceMetrics() {
        // Track maximum current draw
        double current = m_extensionMotor.getOutputCurrent();
        if (current > m_maxCurrent) {
            m_maxCurrent = current;
            System.out.println(">> New max current: " + m_maxCurrent + "A");
        }
        
        // Log extreme values to help with debugging
        if (current > Constants.BALL_ARM_STALL_CURRENT_THRESHOLD) {
            System.out.println(">> HIGH CURRENT DETECTED: " + current + "A");
        }
    }
    
    @Override
    public void periodic() {
        // Update ball detection status
        boolean currentBallStatus = hasBall();
        if (currentBallStatus != m_hasBall) {
            m_hasBall = currentBallStatus;
        }
        
        // Blink status indicator for dashboard
        long now = System.currentTimeMillis();
        if (now - m_lastStatusTime > 250) {
            m_statusBlink = !m_statusBlink;
            m_lastStatusTime = now;
        }
        
        // Track performance for diagnostics
        trackPerformanceMetrics();
        
        // Update SmartDashboard with arm status - CRITICAL FOR DRIVERS
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Extended Limit", !m_extendedLimitSwitch.get());
        SmartDashboard.putBoolean("Retracted Limit", !m_retractedLimitSwitch.get());
        SmartDashboard.putBoolean("Has Ball", m_hasBall);
        SmartDashboard.putBoolean("Status Indicator", m_statusBlink);
        SmartDashboard.putNumber("Ball Distance", m_ballDetector.getRangeInches());
        SmartDashboard.putNumber("Arm Current", m_extensionMotor.getOutputCurrent());
        SmartDashboard.putNumber("Gripper Current", m_gripperMotor.getOutputCurrent());
        SmartDashboard.putNumber("Max Current", m_maxCurrent);
    }
    
    /**
     * Emergency stop all motors
     * When things go wrong, hit this fast!
     */
    public void emergencyStop() {
        // IMMEDIATELY KILL ALL MOTORS
        m_extensionMotor.set(0);
        m_gripperMotor.set(0);
        System.out.println("");
        System.out.println(">> EMERGENCY STOP ACTIVATED!");
        System.out.println(">> ALL ARM SYSTEMS HALTED!");
        System.out.println(">> CHECK FOR MECHANICAL ISSUES!");
        System.out.println("");
    }
}
