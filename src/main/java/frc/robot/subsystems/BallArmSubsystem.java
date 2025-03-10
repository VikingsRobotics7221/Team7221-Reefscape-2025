package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;
import frc.robot.utils.MotorSafetyMonitor;

/**
 * BallArmSubsystem - Controls the robot's arm for ball manipulation.
 * 
 * This subsystem manages an arm with the following capabilities:
 * - Vertical movement (up/down)
 * - Intake/outtake of balls
 * - Position tracking via encoders
 * - Safety limits to prevent mechanical damage
 * 
 * The arm is used to collect balls from the ground, hold them, and place them
 * in scoring locations during the Reefscape 2025 competition.
 */
public class BallArmSubsystem extends SubsystemBase {
    
    // Motors
    private final CANSparkMax armMotor;         // Controls vertical arm movement
    private final CANSparkMax intakeMotor;      // Controls ball intake/outtake
    
    // Sensors
    private final RelativeEncoder armEncoder;   // Tracks arm position
    private final DigitalInput upperLimitSwitch; // Prevents arm from going too high
    private final DigitalInput lowerLimitSwitch; // Prevents arm from going too low
    private final DigitalInput ballDetector;     // Detects when a ball is held
    
    // Controllers
    private final SparkPIDController armPIDController;
    
    // Safety
    private final MotorSafetyMonitor safetyMonitor;
    
    // State tracking
    private boolean hasBall = false;
    private boolean isMoving = false;
    private double targetPosition = 0.0;
    
    /**
     * Creates a new BallArmSubsystem.
     * Initializes all motors, sensors, and controllers with appropriate settings.
     */
    public BallArmSubsystem() {
        // Initialize arm motor
        armMotor = new CANSparkMax(Constants.BALL_ARM_MOTOR_ID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armMotor.setSmartCurrentLimit(Constants.BALL_ARM_CURRENT_LIMIT);
        
        // Initialize intake motor
        intakeMotor = new CANSparkMax(Constants.BALL_INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(Constants.BALL_INTAKE_CURRENT_LIMIT);
        intakeMotor.setInverted(Constants.BALL_INTAKE_INVERTED);
        
        // Configure encoder
        armEncoder = armMotor.getEncoder();
        armEncoder.setPositionConversionFactor(Constants.BALL_ARM_ENCODER_FACTOR);
        armEncoder.setPosition(0);
        
        // Set up PID controller
        armPIDController = armMotor.getPIDController();
        armPIDController.setP(Constants.BALL_ARM_PID_P);
        armPIDController.setI(Constants.BALL_ARM_PID_I);
        armPIDController.setD(Constants.BALL_ARM_PID_D);
        armPIDController.setOutputRange(-Constants.BALL_ARM_MAX_OUTPUT, Constants.BALL_ARM_MAX_OUTPUT);
        
        // Initialize limit switches and ball detector
        upperLimitSwitch = new DigitalInput(Constants.BALL_ARM_UPPER_LIMIT_SWITCH);
        lowerLimitSwitch = new DigitalInput(Constants.BALL_ARM_LOWER_LIMIT_SWITCH);
        ballDetector = new DigitalInput(Constants.BALL_DETECTOR_SWITCH);
        
        // Initialize safety monitor
        safetyMonitor = new MotorSafetyMonitor(
            "Ball Arm", 
            armMotor, 
            Constants.BALL_ARM_MAX_TEMP,
            Constants.BALL_ARM_MAX_CURRENT
        );
        
        // Set initial target position
        targetPosition = armEncoder.getPosition();
        
        // Log initialization
        SmartDashboard.putString("Ball Arm Status", "Initialized");
    }
    
    /**
     * This method is called periodically by the CommandScheduler.
     * It updates sensor readings, checks safety conditions, and 
     * logs information to the dashboard.
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
     * The position is given in encoder units, where 0 is the lowest position.
     * 
     * @param position The target position in encoder units
     */
    public void setArmPosition(double position) {
        // Clamp the position to safe limits
        position = Math.min(Math.max(position, Constants.BALL_ARM_MIN_POSITION), 
                          Constants.BALL_ARM_MAX_POSITION);
        
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
            armMotor.set(Constants.BALL_ARM_UP_SPEED);
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
            armMotor.set(-Constants.BALL_ARM_DOWN_SPEED);
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
        armMotor.set(0);
        isMoving = false;
        targetPosition = armEncoder.getPosition();
        SmartDashboard.putString("Ball Arm Status", "Stopped");
    }
    
    /**
     * Runs the intake to collect a ball.
     * Will automatically stop when a ball is detected.
     */
    public void intakeBall() {
        if (!hasBall) {
            intakeMotor.set(Constants.BALL_INTAKE_SPEED);
            SmartDashboard.putString("Intake Status", "Intaking");
        } else {
            stopIntake();
        }
    }
    
    /**
     * Runs the intake in reverse to expel a ball.
     */
    public void outtakeBall() {
        intakeMotor.set(-Constants.BALL_OUTTAKE_SPEED);
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
     * Typically used when the arm is known to be at a specific position.
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
            if (armMotor.get() > 0 && isAtUpperLimit()) {
                stopArm();
            }
            
            // If moving down and hit lower limit, stop
            if (armMotor.get() < 0 && isAtLowerLimit()) {
                stopArm();
            }
        }
    }
    
    /**
     * Updates the SmartDashboard with current subsystem information.
     * Useful for debugging and driver feedback.
     */
    private void updateDashboard() {
        SmartDashboard.putNumber("Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Has Ball", hasBall());
        SmartDashboard.putBoolean("At Upper Limit", isAtUpperLimit());
        SmartDashboard.putBoolean("At Lower Limit", isAtLowerLimit());
        SmartDashboard.putNumber("Arm Motor Temperature", armMotor.getMotorTemperature());
        SmartDashboard.putNumber("Intake Motor Temperature", intakeMotor.getMotorTemperature());
    }
    
    /**
     * Prepares the arm for the autonomous period.
     * Sets the arm to a known starting position.
     */
    public void prepareForAuto() {
        // Move to the starting position for autonomous
        setArmPosition(Constants.BALL_ARM_AUTO_START_POSITION);
    }
    
    /**
     * Prepares the arm for teleop control.
     * This method can be called when switching from autonomous to teleop.
     */
    public void prepareForTeleop() {
        // Any specific setup for teleop can go here
        stopArm();
        stopIntake();
    }
}
