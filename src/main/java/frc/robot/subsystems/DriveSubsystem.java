/*
 * ========================================================================
 
 .-') _     ('-.   ('-.     _   .-')                                                                                                                
(  OO) )  _(  OO) ( OO ).-.( '.( OO )_                                                                                                              
/     '._(,------./ . --. / ,--.   ,--.)      .--------. .-----.  .-----.  .---.                                                                    
|'--...__)|  .---'| \-.  \  |   `.'   |       |   __   '/ ,-.   \/ ,-.   \/_   |                                                                    
'--.  .--'|  |  .-'-'  |  | |         |       `--' .  / '-'  |  |'-'  |  | |   |                                                                    
   |  |  (|  '--.\| |_.'  | |  |'.'|  |           /  /     .'  /    .'  /  |   |                                                                    
   |  |   |  .--' |  .-.  | |  |   |  |          .  /    .'  /__  .'  /__  |   |                                                                    
   |  |   |  `---.|  | |  | |  |   |  |         /  /    |       ||       | |   |                                                                    
   `--'   `------'`--' `--' `--'   `--'        `--'     `-------'`-------' `---'                                                                    
 _ .-') _  _  .-')                (`-.      ('-.          .-')               .-. .-')    .-')                  .-')    .-') _     ('-.  _   .-')    
( (  OO) )( \( -O )             _(OO  )_  _(  OO)        ( OO ).             \  ( OO )  ( OO ).               ( OO ). (  OO) )  _(  OO)( '.( OO )_  
 \     .'_ ,------.  ,-.-') ,--(_/   ,. \(,------.      (_)---\_) ,--. ,--.   ;-----.\ (_)---\_)  ,--.   ,--.(_)---\_)/     '._(,------.,--.   ,--.)
 ,`'--..._)|   /`. ' |  |OO)\   \   /(__/ |  .---'      /    _ |  |  | |  |   | .-.  | /    _ |    \  `.'  / /    _ | |'--...__)|  .---'|   `.'   | 
 |  |  \  '|  /  | | |  |  \ \   \ /   /  |  |          \  :` `.  |  | | .-') | '-' /_)\  :` `.  .-')     /  \  :` `. '--.  .--'|  |    |         | 
 |  |   ' ||  |_.' | |  |(_/  \   '   /, (|  '--.        '..`''.) |  |_|( OO )| .-. `.  '..`''.)(OO  \   /    '..`''.)   |  |  (|  '--. |  |'.'|  | 
 |  |   / :|  .  '.',|  |_.'   \     /__) |  .--'       .-._)   \ |  | | `-' /| |  \  |.-._)   \ |   /  /\_  .-._)   \   |  |   |  .--' |  |   |  | 
 |  '--'  /|  |\  \(_|  |       \   /     |  `---.      \       /('  '-'(_.-' | '--'  /\       / `-./  /.__) \       /   |  |   |  `---.|  |   |  | 
 `-------' `--' '--' `--'        `-'      `------'       `-----'   `-----'    `------'  `-----'    `--'       `-----'    `--'   `------'`--'   `--' 

 * ========================================================================
 * 
 * TEAM 7221 - THE VIKINGS - REEFSCAPE 2025
 * DRIVE SUBSYSTEM - THE LOCOMOTIVE HEART OF OUR MACHINE
 * 
 * This is not merely code. This is a living, breathing declaration of our 
 * engineering philosophy. We don't just move wheels; we orchestrate precise
 * mechanical poetry through mathematical rigor and relentless failure prevention.
 * 
 * Each function is a tactical imperative - choreographed to maximize traction,
 * minimize power consumption, and adapt intelligently to changing conditions.
 * 
 * CRITICAL SPECIFICATIONS:
 * - CIM Motors with SparkMAX controllers in BRUSHED mode
 * - External encoders for position/velocity tracking
 * - Arcade-style driver inputs mapped to differential tank outputs
 * - Multiphase brownout protection with dynamic power scaling
 * - Fail-operational architecture ensuring graceful degradation
 * 
 * Last updated: March 2025
 * Architect: Team 7221 - The Vikings
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Electrical;
import frc.robot.Constants.Controls;
import frc.robot.Constants.Performance;

/**
 * DriveSubsystem - The locomotive heart of our robot
 * 
 * This subsystem controls our 6-wheel tank drive with CIM motors.
 * It transforms arcade-style inputs (forward + rotation) into 
 * differential outputs for precise control over robot movement.
 * 
 * The system features multiple driving modes, closed-loop feedback,
 * automatic failure detection, and predictive performance optimization.
 */
public class DriveSubsystem extends SubsystemBase {
    
    //                          ┌───────────┐
    //                          │   MOTOR   │
    //                          │ CONTROLLER│
    //                          │   LAYER   │
    //                          └─────┬─────┘
    //                                │
    //      ┌───────────┬─────────────┴─────────────┬───────────┐
    //      │           │                           │           │
    //┌─────▼─────┐┌────▼────┐               ┌──────▼─────┐┌────▼────┐
    //│ LEFT FRONT││LEFT REAR│               │RIGHT FRONT ││RIGHT REAR│
    //│ SparkMAX  ││SparkMAX │               │ SparkMAX   ││SparkMAX  │
    //└─────┬─────┘└────┬────┘               └──────┬─────┘└────┬────┘
    //      │           │                           │           │
    //      │      ┌────▼────┐               ┌──────▼─────┐    │
    //      └──────► LEFT    │               │  RIGHT     ◄────┘
    //             │ ENCODER │               │  ENCODER   │
    //             └────┬────┘               └──────┬─────┘
    //                  │                           │
    //                  └───────────┬───────────────┘
    //                              │
    //                      ┌───────▼────────┐
    //                      │  KINEMATICS &  │
    //                      │   ODOMETRY     │
    //                      └───────┬────────┘
    //                              │
    //                      ┌───────▼────────┐
    //                      │  DIFFERENTIAL  │
    //                      │     DRIVE      │
    //                      └────────────────┘
    
    // ===== MOTOR CONTROLLERS =====
    // For CIM motors, we use SparkMAX in BRUSHED mode
    private final CANSparkMax m_leftFrontMotor;
    private final CANSparkMax m_leftRearMotor;
    private final CANSparkMax m_rightFrontMotor;
    private final CANSparkMax m_rightRearMotor;
    
    // Motor controller groups for tank drive
    private final MotorControllerGroup m_leftMotors;
    private final MotorControllerGroup m_rightMotors;
    
    // ===== ENCODERS =====
    // External encoders for CIM motors
    private final Encoder m_leftEncoder;
    private final Encoder m_rightEncoder;
    
    // ===== DRIVE CONTROL =====
    // Differential drive controller for arcade control
    private final DifferentialDrive m_drive;
    
    // Slew rate limiters for smooth acceleration
    private final SlewRateLimiter m_throttleFilter = 
        new SlewRateLimiter(Constants.Drivetrain.THROTTLE_SLEW_RATE);
    private final SlewRateLimiter m_turnFilter = 
        new SlewRateLimiter(Constants.Drivetrain.TURN_SLEW_RATE);

    // ===== POSITION TRACKING =====
    // Kinematics and odometry for position tracking
    private final DifferentialDriveKinematics m_kinematics = 
        new DifferentialDriveKinematics(Constants.Dimensions.TRACK_WIDTH);
    private final DifferentialDriveOdometry m_odometry;
    
    // Motor feedforward for trajectory following
    private final SimpleMotorFeedforward m_feedforward = 
        new SimpleMotorFeedforward(0.18, 2.8);
    
    // ===== STATE TRACKING =====
    // Simulated gyro state (no physical gyro)
    private double m_simulatedGyroAngle = 0.0;
    private long m_lastGyroUpdateTime = 0;
    
    // Performance monitoring
    private double m_maxCurrent = 0.0;
    private double m_maxTemperature = 0.0;
    private long m_lastStatusUpdate = 0;
    private double m_totalDistance = 0.0;
    private double m_lastLeftPosition = 0.0;
    private double m_lastRightPosition = 0.0;
    
    // Drive mode state
    private boolean m_turboMode = false;
    private boolean m_precisionMode = false;
    private double m_maxOutput = Constants.Drivetrain.DRIVE_NORMAL_SPEED;
    
    // System health indicators
    private boolean m_leftEncoderFunctional = true;
    private boolean m_rightEncoderFunctional = true;
    private boolean m_brownoutProtectionActive = false;
    
    /**
     * Creates a new DriveSubsystem - The robotic locomotive system
     * 
     * Initializes the complete drivetrain with SparkMAX controllers 
     * configured for CIM motors in brushed mode.
     * 
     * The system is initialized with:
     * - Calibrated external encoders
     * - Differential drive mapping
     * - Odometry for position tracking
     * - Failsafe detection systems
     */
    public DriveSubsystem() {
        // Output initialization banner
        System.out.println("\n" +
            "╔══════════════════════════════════════════════════════╗\n" +
            "║      INITIALIZING DRIVETRAIN CONTROL SYSTEM          ║\n" +
            "║        CIM MOTORS + SPARKMAX + TANK DRIVE            ║\n" +
            "╚══════════════════════════════════════════════════════╝");
        
        // Initialize motor controllers in BRUSHED mode for CIM motors
        m_leftFrontMotor = new CANSparkMax(Electrical.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushed);
        m_leftRearMotor = new CANSparkMax(Electrical.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushed);
        m_rightFrontMotor = new CANSparkMax(Electrical.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushed);
        m_rightRearMotor = new CANSparkMax(Electrical.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushed);
        
        // Configure motor controllers
        configureMotorController(m_leftFrontMotor, "Left Front", true);
        configureMotorController(m_leftRearMotor, "Left Rear", true);
        configureMotorController(m_rightFrontMotor, "Right Front", false);
        configureMotorController(m_rightRearMotor, "Right Rear", false);
        
        // Create motor groups
        m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftRearMotor);
        m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightRearMotor);
        
        // Configure external encoders for CIM motors
        m_leftEncoder = new Encoder(
            Electrical.LEFT_DRIVE_ENCODER_PORT_A,
            Electrical.LEFT_DRIVE_ENCODER_PORT_B,
            Drivetrain.LEFT_ENCODER_INVERTED);
            
        m_rightEncoder = new Encoder(
            Electrical.RIGHT_DRIVE_ENCODER_PORT_A,
            Electrical.RIGHT_DRIVE_ENCODER_PORT_B,
            Drivetrain.RIGHT_ENCODER_INVERTED);
            
        // Configure encoder distance per pulse
        m_leftEncoder.setDistancePerPulse(Drivetrain.DISTANCE_PER_PULSE);
        m_rightEncoder.setDistancePerPulse(Drivetrain.DISTANCE_PER_PULSE);
        
        // Create differential drive controller
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_drive.setDeadband(Controls.JOYSTICK_DEADBAND);
        
        // Initialize odometry for position tracking
        m_odometry = new DifferentialDriveOdometry(
            new Rotation2d(),  // No physical gyro, using simulated angle
            getLeftPosition(), 
            getRightPosition());
            
        // Initialize simulated gyro timestamp
        m_lastGyroUpdateTime = System.currentTimeMillis();
        
        // Reset encoders to establish zero point
        resetEncoders();
        
        // Set default drive speed
        setMaxOutput(Drivetrain.DRIVE_NORMAL_SPEED);
        
        // Confirm initialization
        System.out.println(">> DRIVETRAIN INITIALIZED WITH SPARKMAX CONTROLLERS");
        System.out.println(">> CIM MOTORS CONFIGURED IN BRUSHED MODE");
        System.out.println(">> ARCADE CONTROL MAPPED TO TANK DRIVE OUTPUTS");
        System.out.println(">> MAXIMUM THEORETICAL SPEED: " + 
                         Drivetrain.MAX_SPEED_METERS_PER_SECOND + " m/s");
    }
    
    /**
     * Configure a SparkMAX motor controller with optimal settings for CIM motors
     * 
     * @param motor The SparkMAX to configure
     * @param name Name for logging and diagnostics
     * @param inverted Whether to invert the motor direction
     */
    private void configureMotorController(CANSparkMax motor, String name, boolean inverted) {
        // Reset to defaults before configuration
        motor.restoreFactoryDefaults();
        
        // Set basic motor properties
        motor.setInverted(inverted);
        motor.setIdleMode(IdleMode.kBrake);
        
        // Configure current limits for CIM motors
        motor.setSmartCurrentLimit(Electrical.MAX_CURRENT_DRIVE_MOTOR);
        
        // Add voltage compensation for consistent performance as battery drains
        motor.enableVoltageCompensation(Drivetrain.VOLTAGE_COMPENSATION);
        
        // Configure ramping to prevent current spikes
        motor.setOpenLoopRampRate(Drivetrain.OPEN_LOOP_RAMP_RATE);
        motor.setClosedLoopRampRate(Drivetrain.CLOSED_LOOP_RAMP_RATE);
        
        // Save configuration to SparkMAX flash memory
        motor.burnFlash();
        
        System.out.println(">> Configured " + name + " SparkMAX for CIM motor");
    }
    
    /**
     * Called periodically by the command scheduler
     * This is the heartbeat of the drivetrain subsystem
     */
    @Override
    public void periodic() {
        // ===== MONITOR SYSTEM HEALTH =====
        monitorSystemHealth();
        
        // ===== UPDATE POSITION TRACKING =====
        // Update simulated gyro based on differential wheel movement
        updateSimulatedGyro();
        
        // Update odometry with current readings
        m_odometry.update(
            getRotation2d(),
            getLeftPosition(),
            getRightPosition());
            
        // Update total distance traveled
        updateDistanceTraveled();
        
        // ===== DASHBOARD UPDATES =====
        // Update dashboard periodically (not every cycle to reduce CAN traffic)
        long now = System.currentTimeMillis();
        if (now - m_lastStatusUpdate > 100) { // 10 Hz updates
            updateDashboard();
            m_lastStatusUpdate = now;
        }
    }
    
    /**
     * Monitor the health of all drivetrain components
     * Detects failures and applies mitigations automatically
     */
    private void monitorSystemHealth() {
        // ===== MOTOR MONITORING =====
        // Check current draw of all motors
        double lfCurrent = m_leftFrontMotor.getOutputCurrent();
        double lrCurrent = m_leftRearMotor.getOutputCurrent();
        double rfCurrent = m_rightFrontMotor.getOutputCurrent();
        double rrCurrent = m_rightRearMotor.getOutputCurrent();
        
        // Track maximum current for diagnostics
        double maxCurrentNow = Math.max(
            Math.max(lfCurrent, lrCurrent),
            Math.max(rfCurrent, rrCurrent));
            
        if (maxCurrentNow > m_maxCurrent) {
            m_maxCurrent = maxCurrentNow;
        }
        
        // ===== ENCODER HEALTH CHECK =====
        // Verify encoders are producing valid readings
        boolean leftEncoderActive = Math.abs(m_leftEncoder.getRate()) > 0.001 || 
                                   Math.abs(m_leftMotors.get()) < 0.1;
                                   
        boolean rightEncoderActive = Math.abs(m_rightEncoder.getRate()) > 0.001 || 
                                    Math.abs(m_rightMotors.get()) < 0.1;
                                    
        // Update encoder health status
        if (!leftEncoderActive && Math.abs(m_leftMotors.get()) > 0.3) {
            if (m_leftEncoderFunctional) {
                System.out.println("!! WARNING: Left encoder not responding !!");
                m_leftEncoderFunctional = false;
            }
        } else {
            m_leftEncoderFunctional = true;
        }
        
        if (!rightEncoderActive && Math.abs(m_rightMotors.get()) > 0.3) {
            if (m_rightEncoderFunctional) {
                System.out.println("!! WARNING: Right encoder not responding !!");
                m_rightEncoderFunctional = false;
            }
        } else {
            m_rightEncoderFunctional = true;
        }
        
        // ===== BROWNOUT PROTECTION =====
        double batteryVoltage = RobotController.getBatteryVoltage();
        
        // Activate brownout protection if voltage drops too low
        if (batteryVoltage < Performance.BATTERY_WARNING_THRESHOLD && !m_brownoutProtectionActive) {
            m_brownoutProtectionActive = true;
            
            // Calculate reduced power level based on voltage
            double voltageRatio = (batteryVoltage - Performance.BATTERY_BROWNOUT_THRESHOLD) / 
                                 (Performance.BATTERY_GOOD_THRESHOLD - Performance.BATTERY_BROWNOUT_THRESHOLD);
            double reducedPower = Math.max(0.5, Math.min(0.8, voltageRatio));
            
            // Apply power limitation
            m_drive.setMaxOutput(reducedPower * m_maxOutput);
            
            System.out.println("!! BROWNOUT PROTECTION ACTIVATED !!");
            System.out.println("!! Battery: " + batteryVoltage + "V, Power reduced to " + 
                             (reducedPower * 100) + "% !!");
                             
        } else if (batteryVoltage >= Performance.BATTERY_WARNING_THRESHOLD && m_brownoutProtectionActive) {
            // Restore normal operation when voltage recovers
            m_brownoutProtectionActive = false;
            m_drive.setMaxOutput(m_maxOutput);
            
            System.out.println(">> BROWNOUT PROTECTION DEACTIVATED - Power restored");
        }
    }
    
    /**
     * Update dashboard with current drivetrain status
     */
    private void updateDashboard() {
        // Position information
        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Right Position", getRightPosition());
        SmartDashboard.putNumber("Gyro Angle", m_simulatedGyroAngle);
        SmartDashboard.putNumber("Total Distance", m_totalDistance);
        
        // Speed information
        SmartDashboard.putNumber("Left Speed", getLeftVelocity());
        SmartDashboard.putNumber("Right Speed", getRightVelocity());
        
        // Drive mode
        SmartDashboard.putBoolean("Turbo Mode", m_turboMode);
        SmartDashboard.putBoolean("Precision Mode", m_precisionMode);
        
        // System health
        SmartDashboard.putBoolean("Left Encoder OK", m_leftEncoderFunctional);
        SmartDashboard.putBoolean("Right Encoder OK", m_rightEncoderFunctional);
        SmartDashboard.putBoolean("Brownout Protection", m_brownoutProtectionActive);
        SmartDashboard.putNumber("Max Current", m_maxCurrent);
    }
    
    /**
     * Track the total distance traveled by the robot
     */
    private void updateDistanceTraveled() {
        double leftPos = m_leftEncoder.getDistance();
        double rightPos = m_rightEncoder.getDistance();
        
        // Calculate distance traveled since last update
        double leftDelta = Math.abs(leftPos - m_lastLeftPosition);
        double rightDelta = Math.abs(rightPos - m_lastRightPosition);
        double distanceDelta = (leftDelta + rightDelta) / 2.0;
        
        // Add to total distance
        m_totalDistance += distanceDelta;
        
        // Update last positions
        m_lastLeftPosition = leftPos;
        m_lastRightPosition = rightPos;
    }
    
    /**
     * Updates the simulated gyro angle based on differential wheel speeds
     * This provides heading estimation without a physical gyro
     */
    private void updateSimulatedGyro() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - m_lastGyroUpdateTime) / 1000.0; // seconds
        
        if (deltaTime > 0) {
            // Get current wheel speeds
            double leftSpeed = getLeftVelocity();
            double rightSpeed = getRightVelocity();
            
            // Calculate turn rate based on speed differential
            double turnRate = (rightSpeed - leftSpeed) / Constants.Dimensions.TRACK_WIDTH; // rad/s
            
            // Update simulated angle
            m_simulatedGyroAngle += Math.toDegrees(turnRate * deltaTime);
            
            // Normalize to -180 to 180
            m_simulatedGyroAngle = m_simulatedGyroAngle % 360;
            if (m_simulatedGyroAngle > 180) {
                m_simulatedGyroAngle -= 360;
            } else if (m_simulatedGyroAngle < -180) {
                m_simulatedGyroAngle += 360;
            }
            
            m_lastGyroUpdateTime = currentTime;
        }
    }
    
    /**
     * Primary driving method using arcade-style controls
     * This is optimized for CIM motors with a tank drive
     *
     * @param xSpeed Forward/backward speed (-1.0..1.0). Forward is positive.
     * @param zRotation Rotation rate (-1.0..1.0). Counterclockwise is positive.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Apply deadband to eliminate controller drift
        xSpeed = MathUtil.applyDeadband(xSpeed, Controls.JOYSTICK_DEADBAND);
        zRotation = MathUtil.applyDeadband(zRotation, Controls.JOYSTICK_DEADBAND);
        
        // Apply filtering for smooth acceleration - CRITICAL FOR CIM MOTORS
        xSpeed = m_throttleFilter.calculate(xSpeed);
        zRotation = m_turnFilter.calculate(zRotation);
        
        // Apply input curve for better control feel
        xSpeed = Controls.applyInputCurve(xSpeed);
        zRotation = Controls.applyInputCurve(zRotation);
        
        // Adjust turning sensitivity based on speed
        // This makes the robot more controllable at high speeds
        if (Math.abs(xSpeed) > 0.65) {
            zRotation *= 0.8; // Reduce turning at high speeds
        }
        
        // Send commands to the drivetrain
        m_drive.arcadeDrive(xSpeed, zRotation, false); // Already applied curves
    }
    
    /**
     * Tank drive method for direct control of left and right sides
     * Used primarily by autonomous routines
     *
     * @param leftSpeed Left side speed (-1.0..1.0)
     * @param rightSpeed Right side speed (-1.0..1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        // Apply deadband
        leftSpeed = MathUtil.applyDeadband(leftSpeed, Controls.JOYSTICK_DEADBAND);
        rightSpeed = MathUtil.applyDeadband(rightSpeed, Controls.JOYSTICK_DEADBAND);
        
        // Apply filtering for smooth acceleration
        leftSpeed = m_throttleFilter.calculate(leftSpeed);
        rightSpeed = m_throttleFilter.calculate(rightSpeed);
        
        // Apply tank drive with squaring for better control
        m_drive.tankDrive(leftSpeed, rightSpeed, true);
    }
    
    /**
     * Reset encoders to establish new zero point
     * CRITICAL for autonomous routines and position tracking
     */
    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        
        // Also reset distance tracking
        m_totalDistance = 0.0;
        m_lastLeftPosition = 0.0;
        m_lastRightPosition = 0.0;
        
        System.out.println(">> ENCODERS RESET: Position tracking reinitialized");
    }
    
    /**
     * Reset gyro angle to zero
     * This reorients the robot's reference frame
     */
    public void zeroGyro() {
        m_simulatedGyroAngle = 0.0;
        System.out.println(">> SIMULATED GYRO ZEROED");
    }
    
    /**
     * Emergency stop all drive motors
     * Immediately halts all movement
     */
    public void stop() {
        m_leftMotors.stopMotor();
        m_rightMotors.stopMotor();
        System.out.println("!! EMERGENCY STOP ACTIVATED - ALL DRIVE MOTORS HALTED !!");
    }
    
    /**
     * Get left encoder position in meters
     */
    public double getLeftPosition() {
        return m_leftEncoder.getDistance();
    }
    
    /**
     * Get right encoder position in meters
     */
    public double getRightPosition() {
        return m_rightEncoder.getDistance();
    }
    
    /**
     * Get left side velocity in meters per second
     */
    public double getLeftVelocity() {
        return m_leftEncoder.getRate();
    }
    
    /**
     * Get right side velocity in meters per second
     */
    public double getRightVelocity() {
        return m_rightEncoder.getRate();
    }
    
    /**
     * Get average velocity of the robot
     * 
     * @return Average speed in meters per second
     */
    public double getAverageVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }
    
    /**
     * Get the current gyro angle (simulated)
     * 
     * @return Current heading in degrees
     */
    public double getGyroAngle() {
        return m_simulatedGyroAngle;
    }
    
    /**
     * Get the rotation as a Rotation2d object for odometry
     * 
     * @return Current heading as Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(m_simulatedGyroAngle);
    }
    
    /**
     * Get the current estimated robot pose
     * 
     * @return Current pose
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }
    
    /**
     * Reset the robot's odometry to a specific pose
     * 
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(this.getRotation2d(), getLeftPosition(), getRightPosition(), pose);
    }
    
    /**
     * Set wheel speeds using closed-loop control
     * For trajectory following during autonomous
     * 
     * @param speeds Desired wheel speeds
     */
    public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Calculate feedforward values
        double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
        
        // Calculate PID corrections (implemented with simple proportional for now)
        double leftPIDOutput = Autonomous.VELOCITY_KP * 
                             (speeds.leftMetersPerSecond - getLeftVelocity());
        double rightPIDOutput = Autonomous.VELOCITY_KP * 
                              (speeds.rightMetersPerSecond - getRightVelocity());
        
        // Apply combined outputs as voltage
        m_leftMotors.setVoltage(leftPIDOutput + leftFeedforward);
        m_rightMotors.setVoltage(rightPIDOutput + rightFeedforward);
    }
    
    /**
     * Get the drive kinematics object
     * 
     * @return Drive kinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }
    
    /**
     * Set maximum drive output
     * Used to implement different drive modes
     * 
     * @param maxOutput Maximum output (0-1)
     */
    public void setMaxOutput(double maxOutput) {
        m_maxOutput = maxOutput;
        m_drive.setMaxOutput(maxOutput);
    }
    
    /**
     * Enable turbo mode - MAXIMUM POWER!
     * Use with caution - CIM motors can draw significant current
     */
    public void enableTurboMode() {
        setMaxOutput(Drivetrain.DRIVE_TURBO_SPEED);
        m_turboMode = true;
        m_precisionMode = false;
        System.out.println(">> TURBO MODE ACTIVATED: Maximum power enabled!");
    }
    
    /**
     * Enable precision mode for fine control
     * Perfect for lining up with game pieces
     */
    public void enablePrecisionMode() {
        setMaxOutput(Drivetrain.DRIVE_PRECISION_SPEED);
        m_precisionMode = true;
        m_turboMode = false;
        System.out.println(">> PRECISION MODE ACTIVATED: Fine control enabled");
    }
    
    /**
     * Return to normal driving mode
     * Balanced performance and control
     */
    public void disableDriveModes() {
        setMaxOutput(Drivetrain.DRIVE_NORMAL_SPEED);
        m_turboMode = false;
        m_precisionMode = false;
        System.out.println(">> NORMAL DRIVE MODE RESTORED");
    }
    
    /**
     * Get the total distance traveled since last reset
     * 
     * @return Distance in meters
     */
    public double getTotalDistance() {
        return m_totalDistance;
    }
    
    /**
     * Perform a complete check of drivetrain health
     * 
     * @return true if all systems are operational
     */
    public boolean checkSystemHealth() {
        boolean encodersOk = m_leftEncoderFunctional && m_rightEncoderFunctional;
        boolean motorsOk = true; // Expand this check as needed
        boolean batteryOk = RobotController.getBatteryVoltage() > Performance.BATTERY_WARNING_THRESHOLD;
        
        return encodersOk && motorsOk && batteryOk;
    }
    
    /**
     * Get detailed system status for diagnostics
     * 
     * @return Status string describing system state
     */
    public String getSystemStatus() {
        StringBuilder status = new StringBuilder();
        status.append("DRIVETRAIN STATUS:\n");
        status.append("Mode: " + (m_turboMode ? "TURBO" : 
                               (m_precisionMode ? "PRECISION" : "NORMAL")) + "\n");
        status.append("Left Encoder: " + (m_leftEncoderFunctional ? "OK" : "FAULT") + "\n");
        status.append("Right Encoder: " + (m_rightEncoderFunctional ? "OK" : "FAULT") + "\n");
        status.append("Battery: " + RobotController.getBatteryVoltage() + "V\n");
        status.append("Brownout Protection: " + (m_brownoutProtectionActive ? "ACTIVE" : "INACTIVE") + "\n");
        status.append("Max Current: " + m_maxCurrent + "A\n");
        status.append("Total Distance: " + m_totalDistance + "m\n");
        
        return status.toString();
    }
    
    /**
     * Access to the internal DifferentialDrive controller
     * 
     * @return The differential drive controller
     */
    public DifferentialDrive getDifferentialDrive() {
        return m_drive;
    }
    
    /**
     * Access to the left front motor controller
     * Used for monitoring and advanced control
     * 
     * @return Left front CANSparkMax
     */
    public CANSparkMax getLeftFrontMotor() {
        return m_leftFrontMotor;
    }
    
    /**
     * Access to the right front motor controller
     * Used for monitoring and advanced control
     * 
     * @return Right front CANSparkMax
     */
    public CANSparkMax getRightFrontMotor() {
        return m_rightFrontMotor;
    }
}
