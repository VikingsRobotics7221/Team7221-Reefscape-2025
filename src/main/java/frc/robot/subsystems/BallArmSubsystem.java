/*
 * ===============================================================
 *  _____   _    _    _     _____ __  __  _____  _    _ ____  !
 * |  __ \ / \  | |  | |   / ____|  \/  |/ ____|| |  | |  _ \ !
 * | |__) / _ \ | |  | |  | (___ | \  / | (___  | |  | | |_) |!
 * |  ___/ /_\ \| |  | |   \___ \| |\/| |\___ \ | |  | |  _ < !
 * | |  / _____ \ |__| |   ____) | |  | |____) || |__| | |_) |!
 * |_| /_/     \_\____/   |_____/|_|  |_|_____/  \____/|____/ !
 *                                                             !
 * ===============================================================
 * 
 * TEAM 7221 - REEFSCAPE 2025 - BALL ARM SUBSYSTEM
 * 
 * "Grab it, grip it, launch it, win it!"
 * 
 * What does this do?
 *  ______________________      
 * |         /\          |
 * |        /  \         |      <- Our awesome robot
 * |       /____\        |
 * |      |  ʘ ʘ |       |      <- Robot face
 * |      |   ᵕ  |       |
 * |       \____/        |
 * |         ||          |
 * |      {{ARM}}        |      <- This is the ball arm!
 * |    o===||===o       |
 * |    |   ||   |       |
 * |____|___||___|_______|
 * |                     |
 * |_____________________|
 * 
 * Coded with <3 by Team 7221 - 2025
 * LET'S CRUSH THE COMPETITION!
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

public class BallArmSubsystem extends SubsystemBase {
    //------------------------------------------
    // MOTOR CONTROLLERS - POWER THE BEAST!!!!!
    //------------------------------------------
    
    /*  
     *    NEO      NEO 550
     *    MOTOR    MOTOR
     *     ___      ___
     *    |   |    |   |
     *    |___|    |___|
     *      |        |
     *      V        V
     *    MOVES     SPINS
     *    ARM UP    WHEELS
     *    & DOWN    TO GRAB
     */
    private final SparkMax m_armMotor; // NEO for lifting
    private final SparkMax m_gripperMotor; // NEO 550 for ball grabbing
    
    //------------------------------------------
    // SENSORS - THE ROBOT'S FEELERS!!!!!!!
    //------------------------------------------
    
    /*
     *   LIMIT SWITCHES        ULTRASONIC SENSOR
     *      _____                  ____
     *     |     |                |    |====> ))) 
     *     |_____|                |____|
     *       | |                    |
     *       V V                    V
     *     STOPS ARM           DETECTS BALLS
     */
    private final DigitalInput m_upperLimitSwitch; // Keeps arm from destroying itself!
    private final DigitalInput m_lowerLimitSwitch; // Keeps arm from exploding!!
    private final Ultrasonic m_ballDetector; // BALL RADAR!!
    
    //------------------------------------------
    // STATE TRACKING VARIABLES
    //------------------------------------------
    private boolean m_hasBall = false; // Currently holding a ball?
    private double m_armPositionZero = 0.0; // Zero point calibration
    private long m_lastStatusTime = 0; // For status blinking
    private boolean m_statusBlink = false; // For blinking lights effect
    
    /**
     * Creates the AWESOME BALL ARM SUBSYSTEM!!!
     */
    public BallArmSubsystem() {
        System.out.println("🤖 INITIALIZING THE EPIC BALL ARM!!!");
        
        //------------------------------------------
        // INITIALIZE MOTORS - SPARKS FLYING!!!!!
        //------------------------------------------
        m_armMotor = new SparkMax(Constants.BALL_ARM_MOTOR_ID, MotorType.kBrushless);
        m_gripperMotor = new SparkMax(Constants.BALL_GRIPPER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure those motors with COOL SETTINGS!!!
        configureSparkMAX(m_armMotor, Constants.BALL_ARM_MOTOR_INVERTED);
        configureSparkMAX(m_gripperMotor, Constants.BALL_GRIPPER_MOTOR_INVERTED);
        
        //------------------------------------------
        // INITIALIZE SENSORS - ROBOT SENSES!!!!!
        //------------------------------------------
        m_upperLimitSwitch = new DigitalInput(Constants.BALL_ARM_UPPER_LIMIT_SWITCH_PORT);
        m_lowerLimitSwitch = new DigitalInput(Constants.BALL_ARM_LOWER_LIMIT_SWITCH_PORT);
        
        // ULTRASONIC SENSOR FOR BALL DETECTION!!
        m_ballDetector = new Ultrasonic(
            Constants.BALL_DETECTOR_PING_PORT,
            Constants.BALL_DETECTOR_ECHO_PORT
        );
        m_ballDetector.setAutomaticMode(true); // Multiple sensors can run at once!
        
        // RESET THE ARM ENCODER!
        resetArmEncoder();
        
        // SUPER COOL BOOT-UP SEQUENCE
        System.out.println("");
        System.out.println("  /\\_/\\  "); 
        System.out.println(" ( o.o ) BALL ARM ACTIVATED!");
        System.out.println("  > ^ <  READY TO GRAB BALLS!!!");
        System.out.println("");
    }
    
    /**
     * Configures a SparkMAX motor controller with AWESOME settings!
     * 
     * @param motor The motor to configure
     * @param inverted Whether to flip the direction
     */
    private void configureSparkMAX(SparkMax motor, boolean inverted) {
        // BUILD THE CONFIG FOR MAXIMUM PERFORMANCE!!!
        SparkMaxConfig config = new SparkMaxConfig();
        
        // BRAKE MODE IS BEST MODE - NO SLOPPY COASTING ALLOWED!!!
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        
        // TELL THE MOTOR WHO'S BOSS!!
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Make sure current limits are set to PROTECT OUR PRECIOUS MOTORS
        motor.setSmartCurrentLimit(30); // 30 amps is PLENTY
    }
    
    /**
     * Reset the arm encoder position - FRESH START!
     */
    public void resetArmEncoder() {
        // MEMORIZE THE CURRENT POSITION AS "ZERO"
        m_armPositionZero = m_armMotor.getEncoder().getPosition();
        System.out.println("⚡ ARM ENCODER ZEROED! ⚡");
    }
    
    /**
     * Get the arm position in rotations
     * 
     * @return Current arm position in rotations
     */
    public double getArmPosition() {
        // MATH TIME! We subtract the zero point for relative position
        return m_armMotor.getEncoder().getPosition() - m_armPositionZero;
    }
    
    /**
     * Move the arm to pick up or score a ball
     * 
     * @param speed Speed to move the arm (-1.0 to 1.0)
     */
    public void moveArm(double speed) {
        /*
         *   SAFETY FIRST! ⚠️
         *   
         *   Check limit switches before allowing movement!
         *   
         *    UPPER SWITCH
         *        _|_   <-- Stop if we hit this going up
         *         |
         *       ARM
         *         |
         *        _|_   <-- Stop if we hit this going down
         *    LOWER SWITCH
         */
        
        // If we're trying to go up but at the top limit, STOP!
        // Or if we're trying to go down but at the bottom limit, STOP!
        if ((speed > 0 && !m_upperLimitSwitch.get()) || 
            (speed < 0 && !m_lowerLimitSwitch.get())) {
            
            // Add GRAVITY COMPENSATION for smoother movement!
            // Calculate gravity assist based on arm angle
            double gravityCompensation = calculateGravityCompensation();
            
            // Apply speed + gravity compensation (within limits)
            m_armMotor.set(speed + gravityCompensation);
        } else {
            // STOP THE ARM! We've hit a limit!
            m_armMotor.set(0);
        }
    }
    
    /**
     * Calculate gravity compensation for the arm
     * 
     * @return Compensation factor to add to motor output
     */
    private double calculateGravityCompensation() {
        // Get the current arm position to figure out the angle
        double position = getArmPosition();
        
        // For horizontal to above horizontal: need positive assistance 
        if (position > Constants.BALL_ARM_HORIZONTAL_POSITION) {
            return Constants.BALL_ARM_GRAVITY_FF;
        } 
        // For below horizontal: need negative assistance
        else if (position < Constants.BALL_ARM_HORIZONTAL_POSITION - 0.5) {
            return -Constants.BALL_ARM_GRAVITY_FF * 0.5; // Less help going down
        }
        // Near horizontal: smooth transition
        else {
            // Linear interpolation for the transition zone
            double t = (position - (Constants.BALL_ARM_HORIZONTAL_POSITION - 0.5)) / 0.5;
            return Constants.BALL_ARM_GRAVITY_FF * (2 * t - 1) * 0.5;
        }
    }
    
    /**
     * Set the gripper to trap or release a ball
     * 
     * @param speed Speed to run the gripper (-1.0 to 1.0)
     */
    public void setGripper(double speed) {
        // SPIN THOSE WHEELS! GRAB THAT BALL!
        m_gripperMotor.set(speed);
        
        // Debug message if we're in intake or release mode
        if (speed > 0.1) {
            System.out.println("⟳ BALL NOMMING MODE ACTIVE: " + speed);
        } else if (speed < -0.1) {
            System.out.println("⟲ BALL YEETING MODE ACTIVE: " + speed);
        }
    }
    
    /**
     * Check if a ball is detected in the gripper
     * 
     * @return true if ball is detected
     */
    public boolean hasBall() {
        // CHECK THE ULTRASONIC RANGER! HOW FAR IS THE OBJECT?
        double rangeInches = m_ballDetector.getRangeInches();
        
        // If the distance is less than the threshold, BALL DETECTED!
        boolean ballDetected = rangeInches < Constants.BALL_DETECTION_THRESHOLD_INCHES;
        
        // If we have a NEW ball detection, announce it!
        if (ballDetected && !m_hasBall) {
            System.out.println("");
            System.out.println("   ⭐ BALL DETECTED! GOTCHA! ⭐");
            System.out.println("       Distance: " + rangeInches + " inches");
            System.out.println("");
        }
        
        return ballDetected;
    }
    
    /**
     * Set the arm to a specific position using PID control
     * 
     * @param targetPosition Target position in rotations
     */
    public void setArmPosition(double targetPosition) {
        // First SAFETY CHECK - make sure position is within safe boundaries
        if (targetPosition < Constants.BALL_ARM_MIN_POSITION) {
            targetPosition = Constants.BALL_ARM_MIN_POSITION;
            System.out.println("⚠️ ARM POSITION CLAMPED TO MINIMUM!");
        } else if (targetPosition > Constants.BALL_ARM_MAX_POSITION) {
            targetPosition = Constants.BALL_ARM_MAX_POSITION;
            System.out.println("⚠️ ARM POSITION CLAMPED TO MAXIMUM!");
        }
        
        double currentPosition = getArmPosition();
        double error = targetPosition - currentPosition;
        
        // Simple P controller - ERROR FIXING MATH!
        double output = Constants.BALL_ARM_KP * error;
        
        // Clamp the output to safe speed range
        if (output > Constants.BALL_ARM_MAX_SPEED) {
            output = Constants.BALL_ARM_MAX_SPEED;
        } else if (output < -Constants.BALL_ARM_MAX_SPEED) {
            output = -Constants.BALL_ARM_MAX_SPEED;
        }
        
        // Add gravity compensation based on arm angle
        double gravityCompensation = calculateGravityCompensation();
        
        // Combine PID output with gravity compensation
        moveArm(output + gravityCompensation);
        
        // Debug info for significant arm motion
        if (Math.abs(error) > 0.5) {
            // Only log occasionally to avoid spam
            long now = System.currentTimeMillis();
            if (now - m_lastStatusTime > 500) {
                System.out.println("🔄 ARM MOVING TO: " + targetPosition + 
                                   " (current: " + currentPosition + 
                                   ", error: " + error + ")");
                m_lastStatusTime = now;
            }
        }
    }
    
    /**
     * Move the arm to the home position
     */
    public void homeArm() {
        System.out.println("🏠 ARM GOING HOME!");
        setArmPosition(Constants.BALL_ARM_HOME_POSITION);
    }
    
    /**
     * Move the arm to the pickup position
     */
    public void pickupPosition() {
        System.out.println("");
        System.out.println("   ⬇️ ARM GOING TO PICKUP POSITION!");
        System.out.println("   TIME TO GRAB SOME BALLS!!!");
        System.out.println("");
        setArmPosition(Constants.BALL_ARM_PICKUP_POSITION);
    }
    
    /**
     * Move the arm to the scoring position
     */
    public void scorePosition() {
        System.out.println("");
        System.out.println("   ⬆️ ARM GOING TO SCORE POSITION!");
        System.out.println("   PREPARE THE BALL CANNONS!!!");
        System.out.println("");
        setArmPosition(Constants.BALL_ARM_SCORE_POSITION);
    }
    
    /**
     * Automatically intake a ball
     */
    public void intakeBall() {
        // If we don't have a ball, run the gripper to intake
        if (!m_hasBall) {
            setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
            System.out.println("🔄 INTAKE MODE ACTIVATED!");
        } else {
            // If we have a ball, just hold it gently
            setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
            System.out.println("👐 HOLDING BALL GENTLY!");
        }
    }
    
    /**
     * Release the ball - FIRE THE CANNONS!
     */
    public void releaseBall() {
        System.out.println("");
        System.out.println("   🚀 RELEASING THE BALL! YEEEEET! 🚀");
        System.out.println("");
        setGripper(Constants.BALL_GRIPPER_RELEASE_SPEED);
    }
    
    @Override
    public void periodic() {
        // Check if we have a ball and update the status
        boolean currentBallStatus = hasBall();
        
        // If the ball status has changed, log it!
        if (currentBallStatus != m_hasBall) {
            m_hasBall = currentBallStatus;
        }
        
        // Blink status (just for fun)
        long now = System.currentTimeMillis();
        if (now - m_lastStatusTime > 250) {
            m_statusBlink = !m_statusBlink;
            m_lastStatusTime = now;
        }
        
        // Update SmartDashboard with arm status
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Upper Limit", !m_upperLimitSwitch.get());
        SmartDashboard.putBoolean("Lower Limit", !m_lowerLimitSwitch.get());
        SmartDashboard.putBoolean("Has Ball", m_hasBall);
        SmartDashboard.putBoolean("Status Blink", m_statusBlink);
        SmartDashboard.putNumber("Ball Distance", m_ballDetector.getRangeInches());
        SmartDashboard.putNumber("Arm Current", m_armMotor.getOutputCurrent());
        SmartDashboard.putNumber("Gripper Current", m_gripperMotor.getOutputCurrent());
    }
    
    /**
     * Emergency stop all motors - USE IN CASE OF DISASTER!
     */
    public void emergencyStop() {
        m_armMotor.set(0);
        m_gripperMotor.set(0);
        System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
        System.out.println("!!! ALL ARM MOTORS STOPPED  !!!");
    }
}
