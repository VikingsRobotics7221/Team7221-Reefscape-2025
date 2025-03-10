package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.utils.MotorSafetyMonitor;

/**
 * BallArmSubsystem controls the robot's arm for ball manipulation.
 * 
 * This subsystem manages an arm that can:
 * - Move up and down to reach balls at different heights
 * - Intake balls from the field using a roller system
 * - Hold balls securely using a detection sensor
 * - Deposit balls in scoring locations
 * 
 * The subsystem provides methods that are used by:
 * - BallControlCommands.java for manual control during teleop
 * - BallTrackingCommand.java for vision-assisted ball pickup
 * - Autonomous command sequences for scoring during the autonomous period
 */
public class BallArmSubsystem extends SubsystemBase {
    
    // Motors
    private final CANSparkMax armLiftMotor;      // Controls vertical arm movement
    private final CANSparkMax intakeMotor;       // Controls ball intake/outtake
    
    // Sensors
    private final RelativeEncoder armEncoder;    // Tracks arm position
    private final DigitalInput upperLimitSwitch; // Prevents arm from going too high
    private final DigitalInput lowerLimitSwitch; // Prevents arm from going too low
    private final DigitalInput ballDetector;     // Detects when a ball is held
    
    // Controllers
    private final SparkPIDController armPIDController;
    
    // Safety monitoring
    private final MotorSafetyMonitor safetyMonitor;
    
    // State tracking
    private boolean hasBall = false;
    private boolean isMoving = false;
    private double targetPosition = 0.0;
    
    // Constants for this subsystem
    private static final int ARM_LIFT_MOTOR_ID = 10;
    private static final int INTAKE_MOTOR_ID = 11;
    private static final int UPPER_LIMIT_SWITCH_PORT = 0;
    private static final int LOWER_LIMIT_SWITCH_PORT = 1;
    private static final int BALL_DETECTOR_PORT = 2;
    private static final int MOTOR_CURRENT_LIMIT = 30;
    private static final double MAX_MOTOR_TEMPERATURE = 80.0; // Celsius
    private static final double ENCODER_POSITION_FACTOR = 1.0;
    private static final double ARM_UP_SPEED = 0.5;
    private static final double ARM_DOWN_SPEED = 0.3;
    private static final double INTAKE_SPEED = 0.7;
    private static final double OUTTAKE_SPEED = 0.7;
    private static final double ARM_MIN_POSITION = 0.0;
    private static final double ARM_MAX_POSITION = 100.0;
    private static final double PID_P = 0.1;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    private static final double PID_MAX_OUTPUT = 0.7;
    private static final double AUTO_START_POSITION = 20.0;
    
    /**
     * Creates a new BallArmSubsystem.
     * Initializes all motors, sensors, and controllers with appropriate settings.
     */
    public BallArmSubsystem() {
        // Initialize arm lift motor
        armLiftMotor = new CANSparkMax(ARM_LIFT_MOTOR_ID, MotorType.kBrushless);
        armLiftMotor.restoreFactoryDefaults();
        armLiftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armLiftMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        
        // Initialize intake motor
        intakeMotor = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        
        // Configure encoder
        armEncoder = armLiftMotor.getEncoder();
        armEncoder.setPositionConversionFactor(ENCODER_POSITION_FACTOR);
        armEncoder.setPosition(0);
        
        // Set up PID controller
        armPIDController = armLiftMotor.getPIDController();
        armPIDController.setP(PID_P);
        armPIDController.setI(PID_I);
        armPIDController.setD(PID_D);
        armPIDController.setOutputRange(-PID_MAX_OUTPUT, PID_MAX_OUTPUT);
        
        // Initialize limit switches and ball detector
        upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_PORT);
        lowerLimitSwitch = new DigitalInput(LOWER_LIMIT_SWITCH_PORT);
        ballDetector = new DigitalInput(BALL_DETECTOR_PORT);
        
        // Initialize safety monitor
        safetyMonitor = new MotorSafetyMonitor(
            "Ball Arm", 
            armLiftMotor, 
            MAX_MOTOR_TEMPERATURE,
            MOTOR_CURRENT_LIMIT
        );
        
        // Set initial target position
        targetPosition = armEncoder.getPosition();
        
        // Log initialization
        SmartDashboard.putString("Ball Arm Status", "Initialized");
    }
    
    /**
     * This method is called periodically by the CommandScheduler.
     * Updates sensor readings, checks safety conditions, and logs information.
     */
    @Override
    public void periodic() {
        // Update ball detection status
        hasBall = !ballDetector.get(); // Switches are typically active-low
        
        // Check limit switches for safety
        checkLimitSwitches();
        
        // Run the safety monitor
        safetyMonitor.check();
        
        // Update dashboard with current state
        updateDashboard();
    }
    
    /**
     * Sets the arm to move to a specific position.
     * 
     * @param position The target position in encoder units (0 is lowest)
     */
    public void setArmPosition(double position) {
        // Clamp the position to safe limits
        position = Math.min(Math.max(position, ARM_MIN_POSITION), ARM_MAX_POSITION);
        
        // Only change target if it's different
        if (Math.abs(position - targetPosition) > 0.1) {
            targetPosition = position;
            armPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
            isMoving = true;
            SmartDashboard.putString("Ball Arm Status", "Moving to position: " + position);
        }
    }
    
    /**
     * Moves the arm up at a controlled speed.
     * Will stop at the upper limit switch.
     */
    public void moveArmUp() {
        if (!isAtUpperLimit()) {
            armLiftMotor.set(ARM_UP_SPEED);
            isMoving = true;
            SmartDashboard.putString("Ball Arm Status", "Moving Up");
        } else {
            stopArm();
        }
    }
    
    /**
     * Moves the arm down at a controlled speed.
     * Will stop at the lower limit switch.
     */
    public void moveArmDown() {
        if (!isAtLowerLimit()) {
            armLiftMotor.set(-ARM_DOWN_SPEED);
            isMoving = true;
            SmartDashboard.putString("Ball Arm Status", "Moving Down");
        } else {
            stopArm();
        }
    }
    
    /**
     * Stops all arm movement immediately.
     */
    public void stopArm() {
        armLiftMotor.set(0);
        isMoving = false;
        targetPosition = armEncoder.getPosition();
        SmartDashboard.putString("Ball Arm Status", "Stopped");
    }
    
    /**
     * Runs the intake to collect a ball.
     * Automatically stops when a ball is detected.
     */
    public void intakeBall() {
        if (!hasBall) {
            intakeMotor.set(INTAKE_SPEED);
            SmartDashboard.putString("Intake Status", "Intaking");
        } else {
            stopIntake();
        }
    }
    
    /**
     * Runs the intake in reverse to expel a ball.
     */
    public void outtakeBall() {
        intakeMotor.set(-OUTTAKE_SPEED);
        SmartDashboard.putString("Intake Status", "Outtaking");
    }
    
    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMotor.set(0);
        SmartDashboard.putString("Intake Status", "Stopped");
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
     * 
     * @return The current position in encoder units
     */
    public double getArmPosition() {
        return armEncoder.getPosition();
    }
    
    /**
     * Resets the arm encoder to zero at the current position.
     */
    public void resetEncoder() {
        armEncoder.setPosition(0);
        targetPosition = 0;
        SmartDashboard.putString("Ball Arm Status", "Encoder Reset");
    }
    
    /**
     * Checks limit switches and stops the motor if limits are reached.
     * This is a safety feature to prevent mechanical damage.
     */
    private void checkLimitSwitches() {
        if (isMoving) {
            // If moving up and hit upper limit, stop
            if (armLiftMotor.get() > 0 && isAtUpperLimit()) {
                stopArm();
            }
            
            // If moving down and hit lower limit, stop
            if (armLiftMotor.get() < 0 && isAtLowerLimit()) {
                stopArm();
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
        SmartDashboard.putNumber("Arm Motor Temperature", armLiftMotor.getMotorTemperature());
        SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getMotorTemperature());
    }
    
    /**
     * Prepares the arm for the autonomous period.
     * Sets the arm to a known starting position.
     */
    public void prepareForAuto() {
        // Move to the starting position for autonomous
        setArmPosition(AUTO_START_POSITION);
    }
    
    /**
     * Prepares the arm for teleop control.
     */
    public void prepareForTeleop() {
        // Any specific setup for teleop
        stopArm();
        stopIntake();
    }
}
