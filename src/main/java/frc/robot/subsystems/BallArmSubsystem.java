/*
 * ===============================================================
 *  ____  _    _    _    _       ____  _     ___ ____  _____     !
 * |  _ \| |  | |  / \  | |     / ___|| |   |_ _|  _ \| ____|    !
 * | | | | |  | | / _ \ | |     \___ \| |    | || | | |  _|      !
 * | |_| | |__| |/ ___ \| |___   ___) | |___ | || |_| | |___     !
 * |____/|_____/_/   \_\_____|_|____/|_____|___|____/|_____|     !
 *                                                                !
 * ===============================================================
 * 
 * TEAM 7221 - REEFSCAPE 2025 - DUAL DRAWER SLIDE ARM SYSTEM
 * 
 * This is our EPIC ball control system using those heavy-duty drawer slides
 * we just bought! Each one extends independently but works together for
 * MAXIMUM grabbing range and strength!
 * 
 *  ______________________      
 * |         /\          |
 * |        /  \         |     
 * |       /____\        |     
 * |      |  ʘ ʘ |       |     <- Robot face (super intimidating)
 * |      |   ᵕ  |       |
 * |       \____/        |
 * |         ||          |
 * |   [===||===]---->   |     <- Primary drawer slide
 * |   [===||===]---->   |     <- Secondary drawer slide  
 * |____|___||___|_______|
 * |                     |
 * |_____________________|
 * 
 * coded by paysean
 * 
 * BOW BEFORE THE ALMIGHTY 16:1 RATIO!!
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
 * BallArmSubsystem - DUAL DRAWER SLIDE BALL GRABBING MACHINE!
 * 
 * This subsystem controls our dual heavy-duty drawer slides that extend
 * to pick up game pieces and score them with EXTREME precision! The system
 * uses synchronized NEO motors with limit switches so we don't break our
 * beautiful slides, plus an ultrasonic sensor to detect when we've got a ball.
 * 
 * I spent 3 days tuning this and it's gonna DESTROY the competition!!
 */
public class BallArmSubsystem extends SubsystemBase {
    //------------------------------------------
    // MOTOR CONTROLLERS - DRAWER SLIDE POWER!!
    //------------------------------------------
    
    /*  
     *    NEO         NEO        NEO 550
     *    MOTOR       MOTOR      MOTOR 
     *     ___         ___        ___
     *    |   |       |   |      |   |
     *    |___|       |___|      |___|
     *      |           |          |
     *      V           V          V
     *    PRIMARY     SECONDARY   INTAKE
     *    DRAWER      DRAWER      WHEELS
     *    SLIDE       SLIDE       
     */
    private final SparkMax m_primarySlideMotor; // NEO for primary drawer slide
    private final SparkMax m_secondarySlideMotor; // NEO for secondary drawer slide
    private final SparkMax m_gripperMotor; // NEO 550 for ball intake/scoring
    
    //------------------------------------------
    // SENSORS - ROBOT AWARENESS ENHANCERS!
    //------------------------------------------
    
    /*
     *   LIMIT SWITCHES         ULTRASONIC SENSOR
     *      _____                  ____
     *     |     |                |    |====> ))) 
     *     |_____|                |____|
     *       | |                    |
     *       V V                    V
     *    DETECT END POSITIONS   DETECTS BALLS
     *    PREVENT DAMAGE!!       IN GRIPPER
     */
    private final DigitalInput m_primaryExtendedLimitSwitch; // Stops primary drawer at full extension
    private final DigitalInput m_primaryRetractedLimitSwitch; // Detects primary fully retracted
    private final DigitalInput m_secondaryExtendedLimitSwitch; // Stops secondary drawer at full extension
    private final DigitalInput m_secondaryRetractedLimitSwitch; // Detects secondary fully retracted
    private final Ultrasonic m_ballDetector; // DETECTS BALLS WITH SCIENCE!!
    
    //------------------------------------------
    // STATE TRACKING VARIABLES
    //------------------------------------------
    private boolean m_hasBall = false; // Ball detection status
    private double m_primarySlideZero = 0.0; // Primary slide home position
    private double m_secondarySlideZero = 0.0; // Secondary slide home position
    private long m_lastStatusTime = 0; // For status message timing
    private boolean m_statusBlink = false; // For status indicator
    private int m_stallCounter = 0; // Counter to detect if slides are stuck
    
    /**
     * Creates the AWESOME DUAL DRAWER SLIDE BALL ARM SUBSYSTEM!!!
     */
    public BallArmSubsystem() {
        System.out.println("\n" +
                           "======================================================\n" +
                           ">> INITIALIZING DUAL DRAWER SLIDE BALL ARM SUBSYSTEM!!\n" +
                           ">> THE SECRET WEAPON OF TEAM 7221 IS COMING ONLINE!!!!\n" +
                           ">> PREPARE FOR MAXIMUM EXTENSION AND BALL ACQUISITION!!\n" +
                           "======================================================");
        
        //------------------------------------------
        // INITIALIZE MOTORS - MAXIMUM POWER!!!!!
        //------------------------------------------
        m_primarySlideMotor = new SparkMax(Constants.BALL_ARM_PRIMARY_MOTOR_ID, MotorType.kBrushless);
        m_secondarySlideMotor = new SparkMax(Constants.BALL_ARM_SECONDARY_MOTOR_ID, MotorType.kBrushless);
        m_gripperMotor = new SparkMax(Constants.BALL_GRIPPER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure motors with BEST SETTINGS EVER!!!
        configureSparkMAX(m_primarySlideMotor, Constants.BALL_ARM_PRIMARY_MOTOR_INVERTED);
        configureSparkMAX(m_secondarySlideMotor, Constants.BALL_ARM_SECONDARY_MOTOR_INVERTED);
        configureSparkMAX(m_gripperMotor, Constants.BALL_GRIPPER_MOTOR_INVERTED);
        
        //------------------------------------------
        // INITIALIZE SENSORS - ROBOT AWARENESS!!
        //------------------------------------------
        m_primaryExtendedLimitSwitch = new DigitalInput(Constants.BALL_ARM_PRIMARY_EXTENDED_LIMIT_PORT);
        m_primaryRetractedLimitSwitch = new DigitalInput(Constants.BALL_ARM_PRIMARY_RETRACTED_LIMIT_PORT);
        m_secondaryExtendedLimitSwitch = new DigitalInput(Constants.BALL_ARM_SECONDARY_EXTENDED_LIMIT_PORT);
        m_secondaryRetractedLimitSwitch = new DigitalInput(Constants.BALL_ARM_SECONDARY_RETRACTED_LIMIT_PORT);
        
        // ULTRASONIC SENSOR SETUP FOR BALL DETECTION
        m_ballDetector = new Ultrasonic(
            Constants.BALL_DETECTOR_PING_PORT,
            Constants.BALL_DETECTOR_ECHO_PORT
        );
        Ultrasonic.setAutomaticMode(true); // Enable continuous readings
        
        // RESET THE ARM ENCODERS - ZERO POINT CALIBRATION
        resetArmEncoders();
        
        // BOOT-UP SEQUENCE COMPLETE
        System.out.println("");
        System.out.println("     /\\_/\\  "); 
        System.out.println("    ( ^.^ ) DUAL DRAWER SLIDES ONLINE!");
        System.out.println("    /|___|\\  READY TO GRAB BALLS AND DOMINATE!");
        System.out.println("   / |   | \\ ");
        System.out.println("  *  |___|  *");
        System.out.println("");
    }
    
    /**
     * Configures a SparkMAX motor controller with EPIC settings!
     * 
     * @param motor The motor to configure
     * @param inverted Whether to flip direction
     */
    private void configureSparkMAX(SparkMax motor, boolean inverted) {
        // CREATE OPTIMIZED CONFIG
        SparkMaxConfig config = new SparkMaxConfig();
        
        // BRAKE MODE IS CRITICAL FOR DRAWER SLIDES!
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        
        // APPLY CONFIG TO MOTOR CONTROLLER
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Set current limit to protect motors
        try {
            // 30 amps is perfect - enough power without burning motors
            motor.setSmartCurrentLimit(30); 
        } catch (Exception e) {
            System.out.println("!! WARNING: Failed to set current limit! This could damage motors!");
            e.printStackTrace();
        }
    }
    
    /**
     * Reset both drawer slide encoders
     * This is super important to make sure our position tracking is accurate!
     */
    public void resetArmEncoders() {
        // Set current positions as home references
        m_primarySlideZero = m_primarySlideMotor.getEncoder().getPosition();
        m_secondarySlideZero = m_secondarySlideMotor.getEncoder().getPosition();
        System.out.println(">> DRAWER SLIDE ENCODERS ZEROED! TRACKING SYSTEM RESET! >>");
    }
    
    /**
     * Get primary drawer slide position
     * 
     * @return Current primary slide position relative to zero
     */
    public double getPrimarySlidePosition() {
        return m_primarySlideMotor.getEncoder().getPosition() - m_primarySlideZero;
    }
    
    /**
     * Get secondary drawer slide position
     * 
     * @return Current secondary slide position relative to zero
     */
    public double getSecondarySlidePosition() {
        return m_secondarySlideMotor.getEncoder().getPosition() - m_secondarySlideZero;
    }
    
    /**
     * Get average arm position for general positioning
     * 
     * @return Average position of both slides
     */
    public double getArmPosition() {
        return (getPrimarySlidePosition() + getSecondarySlidePosition()) / 2.0;
    }
    
    /**
     * Move primary drawer slide
     * 
     * @param speed Speed to move (-1.0 to 1.0), positive = extend, negative = retract
     */
    public void movePrimarySlide(double speed) {
        // SAFETY FIRST! Check limit switches before moving
        boolean canExtend = m_primaryExtendedLimitSwitch.get(); // Note: switches return true when NOT pressed
        boolean canRetract = m_primaryRetractedLimitSwitch.get();
        
        // Only allow movement if we're not at a limit
        if ((speed > 0 && canExtend) || (speed < 0 && canRetract)) {
            // Apply motor power
            m_primarySlideMotor.set(speed);
        } else {
            // Stop motor if limit reached
            m_primarySlideMotor.set(0);
            
            // Log limit events but avoid spamming console
            if (speed > 0 && !canExtend) {
                long now = System.currentTimeMillis();
                if (now - m_lastStatusTime > 1000) {
                    System.out.println(">> PRIMARY SLIDE: REACHED FULL EXTENSION LIMIT!");
                    m_lastStatusTime = now;
                }
            } else if (speed < 0 && !canRetract) {
                long now = System.currentTimeMillis();
                if (now - m_lastStatusTime > 1000) {
                    System.out.println(">> PRIMARY SLIDE: REACHED FULL RETRACTION LIMIT!");
                    m_lastStatusTime = now;
                }
            }
        }
    }
    
    /**
     * Move secondary drawer slide
     * 
     * @param speed Speed to move (-1.0 to 1.0), positive = extend, negative = retract
     */
    public void moveSecondarySlide(double speed) {
        // SAFETY FIRST! Check limit switches before moving
        boolean canExtend = m_secondaryExtendedLimitSwitch.get(); // Note: switches return true when NOT pressed
        boolean canRetract = m_secondaryRetractedLimitSwitch.get();
        
        // Only allow movement if we're not at a limit
        if ((speed > 0 && canExtend) || (speed < 0 && canRetract)) {
            // Apply motor power
            m_secondarySlideMotor.set(speed);
        } else {
            // Stop motor if limit reached
            m_secondarySlideMotor.set(0);
            
            // Log limit events but avoid spamming console
            if (speed > 0 && !canExtend) {
                long now = System.currentTimeMillis();
                if (now - m_lastStatusTime > 1000) {
                    System.out.println(">> SECONDARY SLIDE: REACHED FULL EXTENSION LIMIT!");
                    m_lastStatusTime = now;
                }
            } else if (speed < 0 && !canRetract) {
                long now = System.currentTimeMillis();
                if (now - m_lastStatusTime > 1000) {
                    System.out.println(">> SECONDARY SLIDE: REACHED FULL RETRACTION LIMIT!");
                    m_lastStatusTime = now;
                }
            }
        }
    }
    
    /**
     * Move both drawer slides together (synchronized)
     * This makes sure they stay even with each other - SUPER IMPORTANT!!
     * 
     * @param speed Speed to move (-1.0 to 1.0), positive = extend, negative = retract
     */
    public void moveArm(double speed) {
        // Check if we need to balance the slides
        double positionDifference = getPrimarySlidePosition() - getSecondarySlidePosition();
        
        // If slides are out of sync by more than threshold, balance them
        if (Math.abs(positionDifference) > Constants.BALL_ARM_ALIGNMENT_THRESHOLD) {
            // Determine which slide needs to catch up
            if (positionDifference > 0) {
                // Primary is ahead, slow it down slightly
                movePrimarySlide(speed * 0.8);
                moveSecondarySlide(speed * 1.2);
            } else {
                // Secondary is ahead, slow it down slightly
                movePrimarySlide(speed * 1.2);
                moveSecondarySlide(speed * 0.8);
            }
        } else {
            // Slides are in sync, move them at same speed
            movePrimarySlide(speed);
            moveSecondarySlide(speed);
        }
        
        // Log significant movement
        if (Math.abs(speed) > 0.2) {
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 1000) {
                System.out.println(">> DRAWER SLIDES " + 
                                  (speed > 0 ? "EXTENDING" : "RETRACTING") + 
                                  " at " + Math.abs(speed) + " power");
                m_lastStatusTime = now;
            }
        }
    }
    
    /**
     * Set the gripper to intake or release balls
     * 
     * @param speed Speed to run the gripper (-1.0 to 1.0)
     */
    public void setGripper(double speed) {
        // SPIN THOSE INTAKE WHEELS!
        m_gripperMotor.set(speed);
        
        // Debug info
        if (speed > 0.1) {
            System.out.println(">> BALL INTAKE MODE ACTIVATED: " + speed);
        } else if (speed < -0.1) {
            System.out.println(">> BALL LAUNCH MODE ACTIVATED: " + speed);
        }
    }
    
    /**
     * Check if ball is detected in the gripper
     * 
     * @return true if ball is detected
     */
    public boolean hasBall() {
        // READ ULTRASONIC SENSOR
        double rangeInches = m_ballDetector.getRangeInches();
        
        // BALL DETECTED if distance is below threshold
        boolean ballDetected = rangeInches < Constants.BALL_DETECTION_THRESHOLD_INCHES;
        
        // Announce new ball detection - CELEBRATE OUR VICTORY!
        if (ballDetected && !m_hasBall) {
            System.out.println("");
            System.out.println("   >> BALL ACQUIRED! LOCKED AND LOADED! >>");
            System.out.println("       Distance: " + rangeInches + " inches");
            System.out.println("       WE GOT IT! WE GOT IT! WE GOT IT!");
            System.out.println("");
        }
        
        return ballDetected;
    }
    
    /**
     * Set drawer slides to specific positions using PID control
     * 
     * @param targetPosition Target position in encoder units
     */
    public void setArmPosition(double targetPosition) {
        // SAFETY CHECK - enforce position limits
        if (targetPosition < Constants.BALL_ARM_MIN_POSITION) {
            targetPosition = Constants.BALL_ARM_MIN_POSITION;
            System.out.println(">> ARM POSITION LIMITED TO MINIMUM EXTENSION!");
        } else if (targetPosition > Constants.BALL_ARM_MAX_POSITION) {
            targetPosition = Constants.BALL_ARM_MAX_POSITION;
            System.out.println(">> ARM POSITION LIMITED TO MAXIMUM EXTENSION!");
        }
        
        // Calculate errors for both slides
        double primaryError = targetPosition - getPrimarySlidePosition();
        double secondaryError = targetPosition - getSecondarySlidePosition();
        
        // Calculate motor outputs with P control
        double primaryOutput = Constants.BALL_ARM_KP * primaryError;
        double secondaryOutput = Constants.BALL_ARM_KP * secondaryError;
        
        // Limit outputs to safe speed range
        primaryOutput = Math.max(-Constants.BALL_ARM_MAX_SPEED, 
                                Math.min(Constants.BALL_ARM_MAX_SPEED, primaryOutput));
        secondaryOutput = Math.max(-Constants.BALL_ARM_MAX_SPEED, 
                                 Math.min(Constants.BALL_ARM_MAX_SPEED, secondaryOutput));
        
        // Apply calculated motor powers
        movePrimarySlide(primaryOutput);
        moveSecondarySlide(secondaryOutput);
        
        // Log significant arm movement
        if (Math.abs(primaryError) > 0.5 || Math.abs(secondaryError) > 0.5) {
            // Limit logging frequency
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 500) {
                System.out.printf(">> DRAWER SLIDES: Moving to %.2f (P: %.2f, S: %.2f)\n", 
                                 targetPosition, getPrimarySlidePosition(), getSecondarySlidePosition());
                m_lastStatusTime = now;
            }
        }
        
        // Check if we're at our target position
        if (Math.abs(primaryError) < 0.2 && Math.abs(secondaryError) < 0.2) {
            // Reset stall counter when we reach target
            m_stallCounter = 0;
        }
    }
    
    /**
     * Move arm to fully retracted home position
     */
    public void homeArm() {
        System.out.println(">> RETRACTING DRAWER SLIDES TO HOME POSITION!");
        setArmPosition(Constants.BALL_ARM_HOME_POSITION);
    }
    
    /**
     * Move arm to pickup position (partial extension)
     */
    public void pickupPosition() {
        System.out.println("");
        System.out.println("   >> EXTENDING DRAWER SLIDES TO PICKUP POSITION!");
        System.out.println("   BALL ACQUISITION MODE ENGAGED!");
        System.out.println("   GET READY TO GRAB THAT BALL!!!");
        System.out.println("");
        setArmPosition(Constants.BALL_ARM_PICKUP_POSITION);
    }
    
    /**
     * Move arm to scoring position (full extension)
     */
    public void scorePosition() {
        System.out.println("");
        System.out.println("   >> EXTENDING DRAWER SLIDES TO MAXIMUM SCORING POSITION!");
        System.out.println("   PREPARE TO LAUNCH BALL INTO TARGET!");
        System.out.println("   THIS IS GONNA BE EPIC!!!");
        System.out.println("");
        setArmPosition(Constants.BALL_ARM_SCORE_POSITION);
    }
    
    /**
     * Automatically intake a ball
     */
    public void intakeBall() {
        // Run intake if no ball detected
        if (!m_hasBall) {
            setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
            System.out.println(">> BALL INTAKE ACTIVATED! SUCKING AT " + 
                              Constants.BALL_GRIPPER_INTAKE_SPEED * 100 + "% POWER!");
        } else {
            // Hold ball securely if already acquired
            setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
            System.out.println(">> HOLDING BALL SECURELY! MAINTAINING GRIP!");
        }
    }
    
    /**
     * Release ball at high speed
     */
    public void releaseBall() {
        System.out.println("");
        System.out.println("   >> RELEASING BALL AT MAXIMUM VELOCITY! >>");
        System.out.println("   >> YEET MODE ACTIVATED! >>");
        System.out.println("");
        setGripper(Constants.BALL_GRIPPER_RELEASE_SPEED);
    }
    
    /**
     * Detect if drawer slides are stalled/stuck
     * This is SUPER important for protecting our expensive slides!!
     * 
     * @return true if a stall is detected
     */
    private boolean detectStall() {
        // Get current draw from motors
        double primaryCurrent = m_primarySlideMotor.getOutputCurrent();
        double secondaryCurrent = m_secondarySlideMotor.getOutputCurrent();
        
        // Check if current is high (indicating stall) but motors still trying to move
        boolean primaryStalled = primaryCurrent > Constants.BALL_ARM_STALL_CURRENT_THRESHOLD && 
                               m_primarySlideMotor.get() != 0;
        boolean secondaryStalled = secondaryCurrent > Constants.BALL_ARM_STALL_CURRENT_THRESHOLD && 
                                 m_secondarySlideMotor.get() != 0;
        
        if (primaryStalled || secondaryStalled) {
            m_stallCounter++;
            if (m_stallCounter > 10) { // Only trigger after sustained stall
                return true;
            }
        } else {
            m_stallCounter = 0; // Reset counter if no stall
        }
        
        return false;
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
        
        // Check for stall condition - SAFETY FIRST!
        if (detectStall()) {
            // Stop motors to prevent damage
            m_primarySlideMotor.set(0);
            m_secondarySlideMotor.set(0);
            
            // Log stall condition
            System.out.println("!!! DRAWER SLIDE STALL DETECTED !!! MOTORS STOPPED FOR PROTECTION!");
        }
        
        // Update dashboard with system status
        SmartDashboard.putNumber("Primary Slide", getPrimarySlidePosition());
        SmartDashboard.putNumber("Secondary Slide", getSecondarySlidePosition());
        SmartDashboard.putNumber("Average Position", getArmPosition());
        
        SmartDashboard.putBoolean("Primary Extended", !m_primaryExtendedLimitSwitch.get());
        SmartDashboard.putBoolean("Primary Retracted", !m_primaryRetractedLimitSwitch.get());
        SmartDashboard.putBoolean("Secondary Extended", !m_secondaryExtendedLimitSwitch.get());
        SmartDashboard.putBoolean("Secondary Retracted", !m_secondaryRetractedLimitSwitch.get());
        
        SmartDashboard.putBoolean("Has Ball", m_hasBall);
        SmartDashboard.putBoolean("Status Indicator", m_statusBlink);
        SmartDashboard.putNumber("Ball Distance", m_ballDetector.getRangeInches());
        
        SmartDashboard.putNumber("Primary Current", m_primarySlideMotor.getOutputCurrent());
        SmartDashboard.putNumber("Secondary Current", m_secondarySlideMotor.getOutputCurrent());
        SmartDashboard.putNumber("Gripper Current", m_gripperMotor.getOutputCurrent());
        
        // Check for drawer slide misalignment
        double positionDifference = Math.abs(getPrimarySlidePosition() - getSecondarySlidePosition());
        if (positionDifference > Constants.BALL_ARM_ALIGNMENT_THRESHOLD) {
            SmartDashboard.putBoolean("DRAWER SLIDE MISALIGNMENT", true);
            // Only log occasionally to avoid spam
            if (now - m_lastStatusTime > 1000) {
                System.out.println(">> WARNING: DRAWER SLIDES MISALIGNED BY " + positionDifference + " UNITS!");
                m_lastStatusTime = now;
            }
        } else {
            SmartDashboard.putBoolean("DRAWER SLIDE MISALIGNMENT", false);
        }
    }
    
    /**
     * Emergency stop all motors - THE PANIC BUTTON!
     */
    public void emergencyStop() {
        // KILL ALL MOTORS IMMEDIATELY!
        m_primarySlideMotor.set(0);
        m_secondarySlideMotor.set(0);
        m_gripperMotor.set(0);
        System.out.println("");
        System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
        System.out.println("!!! ALL ARM SYSTEMS HALTED   !!!");
        System.out.println("!!! CHECK FOR MECHANICAL ISSUES !!!");
        System.out.println("");
    }
}
