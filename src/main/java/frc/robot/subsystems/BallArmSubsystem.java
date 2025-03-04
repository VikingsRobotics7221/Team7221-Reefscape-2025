// src/main/java/frc/robot/subsystems/BallArmSubsystem.java
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

/*
 * ⭐⭐⭐ BALL ARM SUBSYSTEM ⭐⭐⭐
 * coded by paysean 
 * 
 * This is the magical part of our robot that grabs, holds,
 * and YEETS the balls across the field! It's like a T-Rex arm but BETTER.
 * 
 *   _____
 *  /     \
 * |  o o  |
 * |   ∇   |
 *  \_____/  <- Robot face (our robot is cute don't @ me)
 *     |
 *   __|__   <- This is the arm! It moves up and down!
 *  /     \
 * O       O <- These are the wheels that grab the ball!
 * 
 * WHAT THIS DOES:
 * - Controls an arm motor (NEO) to move up/down
 * - Controls a gripper motor to grab/release balls
 * - Uses limit switches so we don't break things
 * - Uses ultrasonic sensor to see if we have a ball
 * 
 * HOW TO USE:
 * - moveArm(speed) - moves the arm up or down
 * - setGripper(speed) - runs the gripper wheels
 * - hasBall() - tells you if we have a ball
 * - setArmPosition(position) - moves arm to specific position
 * 
 * NOTES:
 * - ALWAYS CHECK LIMIT SWITCHES BEFORE MOVING!
 * - NEVER run motors at full speed (they'll explode probably)
 * - If arm is acting weird, check if it's calibrated
 * 
 * OK LET'S GOOOOO!
 */
public class BallArmSubsystem extends SubsystemBase {
    // Motors - THE SPINNY PARTS! 🔄
    private final SparkMax m_armMotor;
    private final SparkMax m_gripperMotor;
    
    // Sensors - THE ROBOT'S EYES AND FEELINGS! 👀
    private final DigitalInput m_upperLimitSwitch;
    private final DigitalInput m_lowerLimitSwitch;
    private final Ultrasonic m_ballDetector;
    
    // State tracking - ROBOT BRAIN STUFF 🧠
    private boolean m_hasBall = false;
    private double m_armPositionZero = 0.0;
    
    /**
     * Creates a new BallArmSubsystem - THE BIRTH OF THE ARM! 🎉
     */
    public BallArmSubsystem() {
        // Initialize motors - WAKE UP MOTORS!
        m_armMotor = new SparkMax(Constants.BALL_ARM_MOTOR_ID, MotorType.kBrushless);
        m_gripperMotor = new SparkMax(Constants.BALL_GRIPPER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure motors - TELL THEM HOW TO BEHAVE!
        configureSparkMAX(m_armMotor, Constants.BALL_ARM_MOTOR_INVERTED);
        configureSparkMAX(m_gripperMotor, Constants.BALL_GRIPPER_MOTOR_INVERTED);
        
        // Initialize sensors - ROBOT SENSES ACTIVATE!
        m_upperLimitSwitch = new DigitalInput(Constants.BALL_ARM_UPPER_LIMIT_SWITCH_PORT);
        m_lowerLimitSwitch = new DigitalInput(Constants.BALL_ARM_LOWER_LIMIT_SWITCH_PORT);
        
        // Setup ultrasonic sensor for ball detection - ECHO ECHO ECHO!
        m_ballDetector = new Ultrasonic(
            Constants.BALL_DETECTOR_PING_PORT,
            Constants.BALL_DETECTOR_ECHO_PORT
        );
        m_ballDetector.setAutomaticMode(true); // Enable automatic mode for multiple ultrasonics
        
        // Reset encoder position - START FROM ZERO!
        resetArmEncoder();
        
        System.out.println("===== BALL ARM SUBSYSTEM INITIALIZED! =====");
        System.out.println("      READY TO GRAB ALL THE BALLS!!! 🏀");
    }
    
    /**
     * Configure a SparkMAX motor controller - MOTOR SETUP TIME!
     */
    private void configureSparkMAX(SparkMax motor, boolean inverted) {
        // OH WAIT GOTTA MAKE SURE WE DON'T BREAK THINGS!!
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // IDLE MODE = BRAKE because we want it to STOP when we tell it to!
        // Not coast because that's how you get a robot to the hospital lol
    }
    
    /**
     * Reset the arm encoder position - AMNESIA TIME!
     */
    public void resetArmEncoder() {
        // FORGET EVERYTHING YOU KNOW ABOUT POSITION!
        m_armPositionZero = m_armMotor.getEncoder().getPosition();
        System.out.println("✨ ARM ENCODER ZEROED! ✨");
    }
    
    /**
     * Get the arm position in rotations - WHERE ARE WE??
     */
    public double getArmPosition() {
        // MATH TIME! Subtract zero point to get relative position
        return m_armMotor.getEncoder().getPosition() - m_armPositionZero;
    }
    
    /**
     * Move the arm to pick up or score a ball - ARM GO BRRR!
     * @param speed Speed to move the arm (-1.0 to 1.0)
     */
    public void moveArm(double speed) {
        // SAFETY FIRST! Check limit switches before moving
        // If we're trying to go up but we're at the top limit, STOP!
        // Or if we're trying to go down but we're at the bottom limit, STOP!
        if ((speed > 0 && !m_upperLimitSwitch.get()) || 
            (speed < 0 && !m_lowerLimitSwitch.get())) {
            m_armMotor.set(speed);
            // ARM GO ZOOM!
        } else {
            m_armMotor.set(0);
            // ARM STOP! (if we hit a limit)
        }
    }
    
    /**
     * Set the gripper to trap or release a ball - GRAB OR YEET!
     * @param speed Speed to run the gripper (-1.0 to 1.0)
     */
    public void setGripper(double speed) {
        // WHIRRRRRR - BALL GRABBING INTENSIFIES!
        m_gripperMotor.set(speed);
    }
    
    /**
     * Check if a ball is detected in the gripper - BALL RADAR ACTIVATE!
     * @return true if ball is detected
     */
    public boolean hasBall() {
        // GET THE RANGE! How far is the thing in front of the ultrasonic?
        double rangeInches = m_ballDetector.getRangeInches();
        
        // If the distance is less than the threshold, A BALL IS HERE!
        return rangeInches < Constants.BALL_DETECTION_THRESHOLD_INCHES;
    }
    
    /**
     * Move arm to a specific position using PID control - PRECISION MODE!
     * @param targetPosition Target position in rotations
     */
    public void setArmPosition(double targetPosition) {
        double currentPosition = getArmPosition();
        double error = targetPosition - currentPosition;
        
        // Simple P controller - ERROR FIXING MATH!
        double output = Constants.BALL_ARM_KP * error;
        
        // Clamp the output - NO GOING TOO FAST!
        if (output > Constants.BALL_ARM_MAX_SPEED) {
            output = Constants.BALL_ARM_MAX_SPEED; // TOO FAST! SLOW DOWN!
        } else if (output < -Constants.BALL_ARM_MAX_SPEED) {
            output = -Constants.BALL_ARM_MAX_SPEED; // TOO FAST BACKWARDS! SLOW DOWN!
        }
        
        // Add a small feed-forward to counteract gravity - FIGHTING PHYSICS!
        // If arm is up, help it stay up. If arm is down, help it stay down.
        if (currentPosition > Constants.BALL_ARM_HORIZONTAL_POSITION) {
            output += Constants.BALL_ARM_GRAVITY_FF; // GRAVITY PULLS DOWN, WE PUSH UP!
        } else {
            output -= Constants.BALL_ARM_GRAVITY_FF; // GRAVITY HELPS DOWN, WE SLOW DOWN!
        }
        
        moveArm(output); // MOVE IT ALREADY!
    }
    
    /**
     * Move the arm to the home position - GO HOME ARM YOU'RE TIRED!
     */
    public void homeArm() {
        setArmPosition(Constants.BALL_ARM_HOME_POSITION);
        System.out.println("🏠 ARM GOING HOME!");
    }
    
    /**
     * Move the arm to the pickup position - FLOOR BALL GRABBING MODE!
     */
    public void pickupPosition() {
        setArmPosition(Constants.BALL_ARM_PICKUP_POSITION);
        System.out.println("⬇️ ARM GOING TO PICKUP POSITION!");
    }
    
    /**
     * Move the arm to the scoring position - SCORING MODE ACTIVATED!
     */
    public void scorePosition() {
        setArmPosition(Constants.BALL_ARM_SCORE_POSITION);
        System.out.println("⬆️ ARM GOING TO SCORE POSITION!");
    }
    
    /**
     * Automatically intake a ball - SUCC MODE!
     */
    public void intakeBall() {
        // If we don't have a ball, run the gripper to intake - HUNGRY ROBOT!
        if (!m_hasBall) {
            setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
        } else {
            // If we have a ball, just hold it - NOM NOM BALL ACQUIRED!
            setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
        }
    }
    
    /**
     * Release the ball - BALL FREEDOM!
     */
    public void releaseBall() {
        setGripper(Constants.BALL_GRIPPER_RELEASE_SPEED);
        System.out.println("🏀 RELEASING THE BALL! BYE BALL!");
    }
    
    @Override
    public void periodic() {
        // Check if we have a ball and update the status - BALL DETECTION LOOP!
        boolean currentBallStatus = hasBall();
        
        // If the ball status has changed, log it - NEW BALL STATE!
        if (currentBallStatus != m_hasBall) {
            m_hasBall = currentBallStatus;
            if (m_hasBall) {
                System.out.println("🎉 BALL DETECTED! GOT IT!");
            } else {
                System.out.println("😢 BALL LOST! WHERE DID IT GO?");
            }
        }
        
        // Update SmartDashboard with arm status - DASHBOARD UPDATING!
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Upper Limit", !m_upperLimitSwitch.get());
        SmartDashboard.putBoolean("Lower Limit", !m_lowerLimitSwitch.get());
        SmartDashboard.putBoolean("Has Ball", m_hasBall);
        SmartDashboard.putNumber("Ball Distance", m_ballDetector.getRangeInches());
    }
}
