// src/main/java/frc/robot/subsystems/DriveSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                                                                           ║
 * ║   ██████╗ ██████╗ ██╗██╗   ██╗███████╗███████╗██╗   ██╗██████╗ ███████╗  ║
 * ║   ██╔══██╗██╔══██╗██║██║   ██║██╔════╝██╔════╝╚██╗ ██╔╝██╔══██╗██╔════╝  ║
 * ║   ██║  ██║██████╔╝██║██║   ██║█████╗  ███████╗ ╚████╔╝ ██████╔╝███████╗  ║
 * ║   ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝  ╚════██║  ╚██╔╝  ██╔══██╗╚════██║  ║
 * ║   ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗███████║   ██║   ██████╔╝███████║  ║
 * ║   ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝╚══════╝   ╚═╝   ╚═════╝ ╚══════╝  ║
 * ║                                                                           ║
 * ║   T E A M  7 2 2 1  -  R E E F S C A P E  D O M I N A T I O N  2 0 2 5    ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 *
 * Core subsystem controlling robot movement with high-precision arcade drive control.
 * 
 * This optimized system creates smooth, intuitive driving while providing multiple
 * control modes for precision maneuvers or high-speed traversal. The implementation
 * focuses on responsiveness and predictability - two critical factors for competitive
 * game piece manipulation.
 * 
 * IMPORTANT NOTE: This version uses standard PWM motor controllers. For SparkMAX
 * support, uncomment the SparkMAX sections and comment out the PWM implementation.
 * 
 * Control Architecture:
 * ┌───────────────┐     ┌───────────────┐     ┌───────────────┐
 * │  Joystick     │────>│  Subsystem    │────>│  Motor        │
 * │  Input        │     │  Processing   │     │  Controllers  │
 * └───────────────┘     └───────────────┘     └───────────────┘
 *                             │
 *                             ▼
 *                       ┌───────────────┐
 *                       │   Telemetry   │
 *                       │   Logging     │
 *                       └───────────────┘
 */
public class DriveSubsystem extends SubsystemBase {
    
    // ===== DRIVE MOTOR CONTROLLERS =====
    
    /* NOTE: This implementation uses PWM motor controllers for compatibility.
     * To use CAN SparkMAX controllers, comment out this section and uncomment
     * the CANSparkMax section below once libraries are available.
     */
    private final PWMVictorSPX leftLeadMotor = new PWMVictorSPX(Constants.Electrical.LEFT_FRONT_DRIVE_MOTOR_ID);
    private final PWMVictorSPX leftFollowMotor = new PWMVictorSPX(Constants.Electrical.LEFT_REAR_DRIVE_MOTOR_ID);
    private final PWMVictorSPX rightLeadMotor = new PWMVictorSPX(Constants.Electrical.RIGHT_FRONT_DRIVE_MOTOR_ID);
    private final PWMVictorSPX rightFollowMotor = new PWMVictorSPX(Constants.Electrical.RIGHT_REAR_DRIVE_MOTOR_ID);
    
    // Motor controller groups to simplify control
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftLeadMotor, leftFollowMotor);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightLeadMotor, rightFollowMotor);
    
    /* 
     * ===== SPARKMAX IMPLEMENTATION (COMMENTED OUT) =====
     * Uncomment this section when REV libraries are available and comment out the PWM section
     *
     * private CANSparkMax leftLeadMotor;
     * private CANSparkMax leftFollowMotor;
     * private CANSparkMax rightLeadMotor;
     * private CANSparkMax rightFollowMotor;
     * 
     * private RelativeEncoder leftEncoder;
     * private RelativeEncoder rightEncoder;
     */

    // DifferentialDrive controller for simplified driving commands
    private final DifferentialDrive differentialDrive;
    
    // Drive mode tracking
    private boolean precisionMode = false;
    private boolean turboMode = false;
    
    // Performance tracking
    private double currentMaxSpeed = Constants.Drivetrain.DRIVE_NORMAL_SPEED;
    private double lastCommandTime = 0;
    private double lastThrottle = 0;
    private double lastRotation = 0;
    
    /**
     * Creates a new DriveSubsystem - the core movement system for the robot.
     * Initializes motors and drive controller with optimal settings.
     */
    public DriveSubsystem() {
        // Configure motor directions (right side is typically inverted for tank drive)
        rightMotors.setInverted(true); 
        
        /* 
         * ===== SPARKMAX CONFIGURATION CODE (COMMENTED OUT) =====
         * This would be used if we had SparkMAX controllers
         * 
         * leftLeadMotor = new CANSparkMax(Constants.Electrical.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
         * leftFollowMotor = new CANSparkMax(Constants.Electrical.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
         * rightLeadMotor = new CANSparkMax(Constants.Electrical.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
         * rightFollowMotor = new CANSparkMax(Constants.Electrical.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
         * 
         * // Configure SparkMAX controllers
         * configureSparkMaxControllers();
         * 
         * // Get encoders from lead motors
         * leftEncoder = leftLeadMotor.getEncoder();
         * rightEncoder = rightLeadMotor.getEncoder();
         * 
         * // Create differential drive with lead motors
         * differentialDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
         */
        
        // Create differential drive with motor groups
        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
        differentialDrive.setDeadband(Constants.Controls.JOYSTICK_DEADBAND);
        
        // Initialize systems
        setupDriveSystem();
        
        System.out.println("\n" +
                           "╔═════════════════════════════════════════════════════╗\n" +
                           "║  DRIVE SUBSYSTEM ONLINE                             ║\n" +
                           "║  × Standard PWM Motors: ACTIVE                      ║\n" +
                           "║  × Arcade Control: ENABLED                          ║\n" +
                           "║  × Performance Monitoring: ACTIVE                   ║\n" +
                           "╚═════════════════════════════════════════════════════╝");
    }
    
    /**
     * Called periodically by the CommandScheduler
     * Updates dashboard, monitors system health, and manages safety features
     */
    @Override
    public void periodic() {
        // Update dashboard with drive information
        SmartDashboard.putBoolean("Precision Mode", precisionMode);
        SmartDashboard.putBoolean("Turbo Mode", turboMode);
        SmartDashboard.putNumber("Drive Speed Factor", currentMaxSpeed);
        
        // Safety monitoring
        monitorSystemHealth();
    }

    /**
     * Sets up drive system parameters and defaults
     */
    private void setupDriveSystem() {
        // Initialize drive mode
        precisionMode = false;
        turboMode = false;
        currentMaxSpeed = Constants.Drivetrain.DRIVE_NORMAL_SPEED;
        
        /* 
         * This would configure SparkMAX settings if we had them
         * Currently disabled as we're using PWM controllers
         */
    }
    
    /* 
     * ===== SPARKMAX CONFIGURATION METHOD (COMMENTED OUT) =====
     * Uncomment when SparkMAX controllers are available
     * 
     * private void configureSparkMaxControllers() {
     *     // Reset to factory defaults for a clean start
     *     leftLeadMotor.restoreFactoryDefaults();
     *     leftFollowMotor.restoreFactoryDefaults();
     *     rightLeadMotor.restoreFactoryDefaults();
     *     rightFollowMotor.restoreFactoryDefaults();
     *     
     *     // Set up followers
     *     leftFollowMotor.follow(leftLeadMotor);
     *     rightFollowMotor.follow(rightLeadMotor);
     *     
     *     // Set idle mode to brake for better control
     *     leftLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
     *     leftFollowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
     *     rightLeadMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
     *     rightFollowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
     *     
     *     // Current limits for motor protection
     *     leftLeadMotor.setSmartCurrentLimit(Constants.Electrical.MAX_CURRENT_DRIVE_MOTOR);
     *     leftFollowMotor.setSmartCurrentLimit(Constants.Electrical.MAX_CURRENT_DRIVE_MOTOR);
     *     rightLeadMotor.setSmartCurrentLimit(Constants.Electrical.MAX_CURRENT_DRIVE_MOTOR);
     *     rightFollowMotor.setSmartCurrentLimit(Constants.Electrical.MAX_CURRENT_DRIVE_MOTOR);
     *     
     *     // Ramp rate for smooth acceleration
     *     leftLeadMotor.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP_RATE);
     *     rightLeadMotor.setOpenLoopRampRate(Constants.Drivetrain.OPEN_LOOP_RAMP_RATE);
     *     
     *     // Burn flash to save settings
     *     leftLeadMotor.burnFlash();
     *     leftFollowMotor.burnFlash();
     *     rightLeadMotor.burnFlash();
     *     rightFollowMotor.burnFlash();
     * }
     */
    
    /**
     * Drives the robot using arcade drive controls.
     * 
     * @param throttle Forward/backward power (-1.0 to 1.0)
     * @param rotation Rotation power (-1.0 to 1.0)
     */
    public void arcadeDrive(double throttle, double rotation) {
        // Apply drive mode modifiers
        throttle *= currentMaxSpeed;
        rotation *= currentMaxSpeed;
        
        // Apply input curve for improved control feel
        throttle = Constants.Controls.applyInputCurve(throttle);
        rotation = Constants.Controls.applyInputCurve(rotation) * 0.8; // Slightly reduce rotation sensitivity
        
        // Apply slew rate limiting for smooth acceleration
        throttle = applyRateLimit(throttle, lastThrottle, Constants.Drivetrain.THROTTLE_SLEW_RATE);
        rotation = applyRateLimit(rotation, lastRotation, Constants.Drivetrain.TURN_SLEW_RATE);
        
        // Store values for next cycle's rate limiting
        lastThrottle = throttle;
        lastRotation = rotation;
        
        // Update the drive controller
        differentialDrive.arcadeDrive(throttle, rotation);
        lastCommandTime = System.currentTimeMillis();
    }
    
    /**
     * Drives the robot using tank drive controls (direct left/right side control).
     * 
     * @param leftSpeed Left motors speed (-1.0 to 1.0)
     * @param rightSpeed Right motors speed (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        // Apply drive mode modifiers
        leftSpeed *= currentMaxSpeed;
        rightSpeed *= currentMaxSpeed;
        
        // Apply input curve for improved control feel
        leftSpeed = Constants.Controls.applyInputCurve(leftSpeed);
        rightSpeed = Constants.Controls.applyInputCurve(rightSpeed);
        
        // Update the drive controller
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
        lastCommandTime = System.currentTimeMillis();
    }
    
    /**
     * Stops all drive motors immediately.
     * Use this for emergency stops or when precision ending is required.
     */
    public void stopMotors() {
        differentialDrive.stopMotor();
        System.out.println(">> EMERGENCY STOP: Drive motors halted!");
    }
    
    /**
     * Gets the approximate distance traveled.
     * This is a simplified implementation for PWM controllers without encoders.
     * 
     * @return Estimated distance in meters (always 0 for PWM motors without encoders)
     */
    public double getEstimatedDistance() {
        // PWM controllers don't have built-in encoders, so return 0
        // With SparkMAX, we would return actual encoder distance
        return 0.0;
    }
    
    /**
     * Resets the position tracking to zero.
     * For PWM controllers, this does nothing.
     */
    public void resetEncoders() {
        // For PWM controllers without encoders, this is a no-op
        // With SparkMAX, we would reset the encoders
        System.out.println(">> Encoder reset requested (No encoders with PWM controllers)");
    }
    
    /**
     * Enables precision mode (reduced speed for fine control)
     */
    public void enablePrecisionMode() {
        // Disable other modes first
        disableDriveModes();
        
        // Enable precision mode
        precisionMode = true;
        currentMaxSpeed = Constants.Drivetrain.DRIVE_PRECISION_SPEED;
        System.out.println(">> Precision mode activated: " + (currentMaxSpeed * 100) + "% power");
    }
    
    /**
     * Enables turbo mode (maximum speed)
     */
    public void enableTurboMode() {
        // Disable other modes first
        disableDriveModes();
        
        // Enable turbo mode
        turboMode = true;
        currentMaxSpeed = Constants.Drivetrain.DRIVE_TURBO_SPEED;
        System.out.println(">> TURBO MODE ACTIVATED: " + (currentMaxSpeed * 100) + "% power");
    }
    
    /**
     * Disables all special drive modes, returning to normal speed
     */
    public void disableDriveModes() {
        precisionMode = false;
        turboMode = false;
        currentMaxSpeed = Constants.Drivetrain.DRIVE_NORMAL_SPEED;
        System.out.println(">> Drive modes reset: " + (currentMaxSpeed * 100) + "% power");
    }
    
    /**
     * Applies rate limiting to a control value to prevent sudden changes.
     * 
     * @param current Desired value
     * @param previous Previous value
     * @param rateLimit Maximum change per second
     * @return Rate-limited value
     */
    private double applyRateLimit(double current, double previous, double rateLimit) {
        double maxChange = rateLimit * 0.02; // Assuming 50Hz update rate (0.02 seconds)
        double change = current - previous;
        
        if (change > maxChange) {
            return previous + maxChange;
        } else if (change < -maxChange) {
            return previous - maxChange;
        } else {
            return current;
        }
    }
    
    /**
     * Monitors system health and reports any issues.
     * Limited functionality with PWM controllers.
     */
    private void monitorSystemHealth() {
        // For PWM controllers, we have limited health monitoring
        // With SparkMAX, we would check temperatures, currents, etc.
        
        // Check for command timeout (motors running without updates)
        if (System.currentTimeMillis() - lastCommandTime > 500) {
            // Motors have been running for more than 500ms without a new command
            // This is normal during idle periods, so no action needed
        }
    }
    
    /**
     * Returns the current drive subsystem state as a debug string.
     * Used for logging and debugging.
     * 
     * @return String representation of drive state
     */
    public String getDriveStateInfo() {
        StringBuilder info = new StringBuilder();
        info.append("Drive State: ");
        
        if (precisionMode) {
            info.append("PRECISION MODE, ");
        } else if (turboMode) {
            info.append("TURBO MODE, ");
        } else {
            info.append("NORMAL MODE, ");
        }
        
        info.append(String.format("Speed: %.1f%%", currentMaxSpeed * 100));
        
        return info.toString();
    }
    
    /**
     * Gets whether the drive is currently in precision mode
     * 
     * @return true if precision mode is active
     */
    public boolean isPrecisionMode() {
        return precisionMode;
    }
    
    /**
     * Gets whether the drive is currently in turbo mode
     * 
     * @return true if turbo mode is active
     */
    public boolean isTurboMode() {
        return turboMode;
    }
}
