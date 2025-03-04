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

/**
 * Ball Control Arm Subsystem
 * Controls the mechanism for trapping, holding, and releasing game pieces
 */
public class BallArmSubsystem extends SubsystemBase {
    // Motors
    private final SparkMax m_armMotor;
    private final SparkMax m_gripperMotor;
    
    // Sensors
    private final DigitalInput m_upperLimitSwitch;
    private final DigitalInput m_lowerLimitSwitch;
    private final Ultrasonic m_ballDetector;
    
    // State tracking
    private boolean m_hasBall = false;
    private double m_armPositionZero = 0.0;
    
    /**
     * Creates a new BallArmSubsystem
     */
    public BallArmSubsystem() {
        // Initialize motors
        m_armMotor = new SparkMax(Constants.BALL_ARM_MOTOR_ID, MotorType.kBrushless);
        m_gripperMotor = new SparkMax(Constants.BALL_GRIPPER_MOTOR_ID, MotorType.kBrushless);
        
        // Configure motors
        configureSparkMAX(m_armMotor, Constants.BALL_ARM_MOTOR_INVERTED);
        configureSparkMAX(m_gripperMotor, Constants.BALL_GRIPPER_MOTOR_INVERTED);
        
        // Initialize sensors
        m_upperLimitSwitch = new DigitalInput(Constants.BALL_ARM_UPPER_LIMIT_SWITCH_PORT);
        m_lowerLimitSwitch = new DigitalInput(Constants.BALL_ARM_LOWER_LIMIT_SWITCH_PORT);
        
        // Setup ultrasonic sensor for ball detection
        m_ballDetector = new Ultrasonic(
            Constants.BALL_DETECTOR_PING_PORT,
            Constants.BALL_DETECTOR_ECHO_PORT
        );
        m_ballDetector.setAutomaticMode(true); // Enable automatic mode for multiple ultrasonics
        
        // Reset encoder position
        resetArmEncoder();
    }
    
    /**
     * Configure a SparkMAX motor controller
     */
    private void configureSparkMAX(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    /**
     * Reset the arm encoder position
     */
    public void resetArmEncoder() {
        m_armPositionZero = m_armMotor.getEncoder().getPosition();
    }
    
    /**
     * Get the arm position in rotations
     */
    public double getArmPosition() {
        return m_armMotor.getEncoder().getPosition() - m_armPositionZero;
    }
    
    /**
     * Move the arm to pick up or score a ball
     * @param speed Speed to move the arm (-1.0 to 1.0)
     */
    public void moveArm(double speed) {
        // Check limit switches before moving
        if ((speed > 0 && !m_upperLimitSwitch.get()) || 
            (speed < 0 && !m_lowerLimitSwitch.get())) {
            m_armMotor.set(speed);
        } else {
            m_armMotor.set(0);
        }
    }
    
    /**
     * Set the gripper to trap or release a ball
     * @param speed Speed to run the gripper (-1.0 to 1.0)
     */
    public void setGripper(double speed) {
        m_gripperMotor.set(speed);
    }
    
    /**
     * Check if a ball is detected in the gripper
     * @return true if ball is detected
     */
    public boolean hasBall() {
        // Get the range in inches from the ultrasonic sensor
        double rangeInches = m_ballDetector.getRangeInches();
        
        // If the distance is less than the threshold, a ball is present
        return rangeInches < Constants.BALL_DETECTION_THRESHOLD_INCHES;
    }
    
    /**
     * Move arm to a specific position using PID control
     * @param targetPosition Target position in rotations
     */
    public void setArmPosition(double targetPosition) {
        double currentPosition = getArmPosition();
        double error = targetPosition - currentPosition;
        
        // Simple P controller
        double output = Constants.BALL_ARM_KP * error;
        
        // Clamp the output
        if (output > Constants.BALL_ARM_MAX_SPEED) {
            output = Constants.BALL_ARM_MAX_SPEED;
        } else if (output < -Constants.BALL_ARM_MAX_SPEED) {
            output = -Constants.BALL_ARM_MAX_SPEED;
        }
        
        // Add a small feed-forward to counteract gravity
        if (currentPosition > Constants.BALL_ARM_HORIZONTAL_POSITION) {
            output += Constants.BALL_ARM_GRAVITY_FF;
        } else {
            output -= Constants.BALL_ARM_GRAVITY_FF;
        }
        
        moveArm(output);
    }
    
    /**
     * Move the arm to the home position
     */
    public void homeArm() {
        setArmPosition(Constants.BALL_ARM_HOME_POSITION);
    }
    
    /**
     * Move the arm to the pickup position
     */
    public void pickupPosition() {
        setArmPosition(Constants.BALL_ARM_PICKUP_POSITION);
    }
    
    /**
     * Move the arm to the scoring position
     */
    public void scorePosition() {
        setArmPosition(Constants.BALL_ARM_SCORE_POSITION);
    }
    
    /**
     * Automatically intake a ball
     */
    public void intakeBall() {
        // If we don't have a ball, run the gripper to intake
        if (!m_hasBall) {
            setGripper(Constants.BALL_GRIPPER_INTAKE_SPEED);
        } else {
            // If we have a ball, stop the gripper
            setGripper(Constants.BALL_GRIPPER_HOLD_SPEED);
        }
    }
    
    /**
     * Release the ball
     */
    public void releaseBall() {
        setGripper(Constants.BALL_GRIPPER_RELEASE_SPEED);
    }
    
    @Override
    public void periodic() {
        // Check if we have a ball and update the status
        boolean currentBallStatus = hasBall();
        
        // If the ball status has changed, log it
        if (currentBallStatus != m_hasBall) {
            m_hasBall = currentBallStatus;
            System.out.println("Ball status changed: " + (m_hasBall ? "Ball detected" : "No ball detected"));
        }
        
        // Update SmartDashboard with arm status
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Upper Limit", !m_upperLimitSwitch.get());
        SmartDashboard.putBoolean("Lower Limit", !m_lowerLimitSwitch.get());
        SmartDashboard.putBoolean("Has Ball", m_hasBall);
        SmartDashboard.putNumber("Ball Distance", m_ballDetector.getRangeInches());
    }
}
