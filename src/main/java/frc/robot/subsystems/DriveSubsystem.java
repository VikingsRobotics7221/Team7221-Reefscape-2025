// DriveSubsystem.java - TOTALLY UPGRADED FOR 16:1 POWER!
package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * THE SUPER-POWERFUL 16:1 DRIVETRAIN SYSTEM!!! 
 * 
 * OUR TANK DRIVE FINALLY HAS THE TORQUE TO CRUSH THE COMPETITION!
 * Updated with improved slew rate for smoother acceleration and
 * adjusted PID tuning for more responsive control!
 * 
 * coded by paysean
 */
public class DriveSubsystem extends SubsystemBase {
  
    // Drivetrain Motor Controllers
    private static SparkMax m_leftFrontMotor; // NEO motor with 16:1 gearing
    private static SparkMax m_rightFrontMotor; // NEO motor with 16:1 gearing
    private static SparkMax m_leftBackMotor; // NEO motor with 16:1 gearing
    private static SparkMax m_rightBackMotor; // NEO motor with 16:1 gearing

    // Motor controller groups for left and right sides
    private MotorControllerGroup m_leftMotors;
    private MotorControllerGroup m_rightMotors;

    // Differential drive for tank drive with arcade control
    private DifferentialDrive m_drive;

    // Slew rate limiters - RETUNED FOR 16:1!!! 
    // Lower values = smoother acceleration but less responsive
    // Higher values = more responsive but jerkier
    private final SlewRateLimiter throttleFilter = new SlewRateLimiter(3.5); // Increased for better response
    private final SlewRateLimiter turnFilter = new SlewRateLimiter(3.25);    // Slightly reduced for smoother turns

    // Drive constants - CRITICAL WITH 16:1 RATIO!
    private final double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO; // 16:1 ratio!!

    // Variables for encoder tracking
    private double leftFrontPositionZero = 0.0; 
    private double rightFrontPositionZero = 0.0;
    private double leftBackPositionZero = 0.0;
    private double rightBackPositionZero = 0.0;
    
    // Variables for turn simulation (replacing gyro)
    private double m_simulatedAngle = 0.0;
    private long m_lastUpdateTime = 0;

    // Track width for kinematics
    private static final double TRACK_WIDTH = Constants.TRACK_WIDTH;

    // Feedforward and PID controllers - RETUNED FOR 16:1 RATIO!
    private static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.18, 2.85, 0.48);
    private static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
            Constants.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
            Constants.kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    
    // PID controllers for velocity tracking
    private final PIDController leftPIDController = new PIDController(Constants.kP_FRONT_LEFT_VELOCITY, 0, 0);
    private final PIDController rightPIDController = new PIDController(Constants.kP_FRONT_RIGHT_VELOCITY, 0, 0);

    // Kinematics and odometry
    private static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);
    private static DifferentialDriveOdometry odometry;
  
    /** 
     * Creates THE BEAST - our 16:1 MONSTER drivetrain!!
     */
    public DriveSubsystem() {
        System.out.println(">> INITIALIZING 16:1 BEAST MODE DRIVETRAIN - PREPARE FOR DOMINANCE!! <<");
        
        // Instantiate the Drivetrain motors with the SUPER-TORQUEY 16:1 gearing
        m_leftFrontMotor = new SparkMax(Constants.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_rightFrontMotor = new SparkMax(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_leftBackMotor = new SparkMax(Constants.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_rightBackMotor = new SparkMax(Constants.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);

        // SUPER IMPORTANT TO CONFIGURE CORRECTLY WITH 16:1 RATIO!!
        configureSparkMAX(m_leftFrontMotor, Constants.REVERSE_LEFT_FRONT_MOTOR);
        configureSparkMAX(m_leftBackMotor, Constants.REVERSE_LEFT_BACK_MOTOR);
        configureSparkMAX(m_rightBackMotor, Constants.REVERSE_RIGHT_BACK_MOTOR);
        configureSparkMAX(m_rightFrontMotor, Constants.REVERSE_RIGHT_FRONT_MOTOR);

        // Create motor controller groups for the tank drive
        m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
        m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

        // Create differential drive (tank drive with arcade control)
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        
        // Set deadband to reduce small unwanted movements from joystick noise
        m_drive.setDeadband(0.05);

        // Create odometry without gyro - using encoder-only odometry
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(),  // No gyro, so use dummy rotation
            0, 0);

        // Zero the drive encoders
        resetEncoders();

        // Initialize the timestamp for gyro simulation
        m_lastUpdateTime = System.currentTimeMillis();

        System.out.println(">> 16:1 DRIVETRAIN ONLINE - MAXIMUM TORQUE READY!!! <<");
        
        // Print configuration details
        System.out.println("     - Drive Gear Ratio: 16:1");
        System.out.println("     - Max Speed: " + Constants.MAX_SAFE_SPEED + " m/s");
        System.out.println("     - Turbo Mode: " + Constants.DRIVE_TURBO_SPEED * 100 + "% power");
    }

    /**
     * Configures a Spark MAX controller with optimal settings for 16:1 gearing
     * 
     * @param max Motor controller to configure
     * @param reverse Whether to invert motor direction
     */
    private void configureSparkMAX(SparkMax max, boolean reverse) {
        // Build a config with our specific settings
        SparkMaxConfig config = new SparkMaxConfig();
        
        // BRAKE MODE IS CRITICAL for 16:1 ratio to prevent coasting!
        config.inverted(reverse).idleMode(IdleMode.kBrake);
        
        // Apply configuration to motor controller
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Set current limit to protect motors with high-torque 16:1 ratio
        try {
            // Current limiting is critical with 16:1 ratio to prevent brownouts!
            // Note: Using REV's 2025 API format. Adjust if version differs.
            // 30 amps is sufficient protection for NEO motors with 16:1 gearing
            max.setSmartCurrentLimit(30); 
        } catch (Exception e) {
            System.out.println("⚠️ WARNING: Failed to set current limit! Protect your motors!");
            e.printStackTrace();
        }
    }

    /**
     * Emergency stop all motors
     */
    public void stop() {
        m_drive.stopMotor();
    }

    /**
     * Reset gyro heading to zero
     */
    public void zeroGyro() {
        // Reset our simulated angle
        m_simulatedAngle = 0.0;
    }
    
    // GYRO SIMULATION METHODS - These simulate a gyro using encoder data
    public double getYaw() {
        return m_simulatedAngle; // Return our simulated angle
    }
    
    public double getPitch() {
        return 0.0; // No pitch detection without real gyro
    }
    
    public double getRoll() {
        return 0.0; // No roll detection without real gyro
    }
    
    /**
     * Gets the simulated gyro angle
     * 
     * @return Current estimated heading in degrees
     */
    public double getGyroAngle() {
        return m_simulatedAngle;
    }
    
    /**
     * Current turn rate (degrees per second)
     * 
     * @return Estimated turn rate
     */
    public double getTurnRate() {
        // Estimate based on differential wheel speeds
        double leftSpeed = getLeftSpeed();
        double rightSpeed = getRightSpeed();
        double turnRate = Math.toDegrees((rightSpeed - leftSpeed) / TRACK_WIDTH);
        return turnRate;
    }

    /**
     * Get the rotation as a Rotation2d object
     * 
     * @return Current heading as Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(m_simulatedAngle);
    }
    
    /**
     * Reset the robot's odometry to a specific pose
     * 
     * @param pose The pose to reset to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(this.getRotation2d(), getLeftPosition(), getRightPosition(), pose);
    }
    
    /**
     * Get the current estimated robot pose
     * 
     * @return Current pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    /**
     * Get the drive kinematics object
     * 
     * @return Drive kinematics
     */
    public DifferentialDriveKinematics getkDriveKinematics() {
        return kDriveKinematics;    
    }
    
    /**
     * Get the theta controller constraints
     * 
     * @return Theta controller constraints
     */
    public TrapezoidProfile.Constraints getkThetaControllerConstraints() {
        return kThetaControllerConstraints;
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(this.getRotation2d(), getLeftPosition(), getRightPosition());

        // Update our simulated gyro based on differential wheel movement
        updateSimulatedGyro();

        // Log drive information to SmartDashboard
        SmartDashboard.putNumber("Left Front Position", getLeftFrontPosition());
        SmartDashboard.putNumber("Right Front Position", getRightFrontPosition());
        SmartDashboard.putNumber("Left Back Position", getLeftBackPosition());
        SmartDashboard.putNumber("Right Back Position", getRightBackPosition());
        
        // Also log velocities for tuning and debugging
        SmartDashboard.putNumber("Left Speed (m/s)", getLeftSpeed());
        SmartDashboard.putNumber("Right Speed (m/s)", getRightSpeed());
        
        // Log simulated angle for debugging
        SmartDashboard.putNumber("Simulated Angle", m_simulatedAngle);
        
        // Log motor current - SUPER IMPORTANT WITH 16:1 RATIO!
        SmartDashboard.putNumber("Left Front Current", m_leftFrontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Front Current", m_rightFrontMotor.getOutputCurrent());
    }
    
    /**
     * Estimates rotation based on differential wheel speeds
     * CRITICAL for navigation without a gyro!
     */
    private void updateSimulatedGyro() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - m_lastUpdateTime) / 1000.0; // seconds
        
        if (deltaTime > 0) {
            // Get current wheel speeds
            double leftSpeed = getLeftSpeed();
            double rightSpeed = getRightSpeed();
            
            // Calculate turn rate based on speed differential
            double turnRate = (rightSpeed - leftSpeed) / TRACK_WIDTH; // rad/s
            
            // Update simulated angle
            m_simulatedAngle += Math.toDegrees(turnRate * deltaTime);
            
            // Normalize to -180 to 180
            m_simulatedAngle = m_simulatedAngle % 360;
            if (m_simulatedAngle > 180) {
                m_simulatedAngle -= 360;
            } else if (m_simulatedAngle < -180) {
                m_simulatedAngle += 360;
            }
            
            m_lastUpdateTime = currentTime;
        }
    }

    /**
     * Arcade drive method for our 16:1 tank-drive hardware.
     * REBALANCED FOR 16:1 GEARING!
     * 
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Apply filtering for smooth acceleration
        xSpeed = throttleFilter.calculate(xSpeed);
        zRotation = turnFilter.calculate(zRotation);
        
        // With 16:1 ratio, we need to be extra careful about sudden changes in direction
        // The square inputs option helps with fine control
        m_drive.arcadeDrive(xSpeed, zRotation, true);
    }

    /**
     * Tank drive method for direct control of left and right sides.
     * 
     * @param leftSpeed Left side speed
     * @param rightSpeed Right side speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        // Apply filtering for smooth acceleration
        leftSpeed = throttleFilter.calculate(leftSpeed);
        rightSpeed = throttleFilter.calculate(rightSpeed);
        
        m_drive.tankDrive(leftSpeed, rightSpeed, true);
    }

    /** 
     * Get encoder positions
     * Adjusted calculations based on 16:1 gearing
     */
    public double getLeftFrontPosition() {
        return (m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftFrontPositionZero);
    }
    
    public double getRightFrontPosition() {
        return -1 * (m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightFrontPositionZero);
    }
    
    public double getLeftBackPosition() {
        return -1 * (m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftBackPositionZero);
    }
    
    public double getRightBackPosition() {
        return (m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightBackPositionZero);
    }
    
    /** 
     * Get motor speeds in RPM
     * Adjusted for 16:1 gearing
     */
    public double getLeftFrontSpeed() {
        return (m_leftFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }
    
    public double getRightFrontSpeed() {
        return -1 * (m_rightFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }
    
    public double getLeftBackSpeed() {
        return -1 * (m_leftBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }
    
    public double getRightBackSpeed() {
        return (m_rightBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }

    // Combined positions for odometry
    public double getLeftPosition() {
        return (getLeftFrontPosition() + getLeftBackPosition()) / 2.0;
    }

    public double getRightPosition() {
        return (getRightFrontPosition() + getRightBackPosition()) / 2.0;
    }

    /**
     * Zero the drivetrain encoders - CRITICAL FOR AUTONOMOUS!!
     */
    public void resetEncoders() {
        leftFrontPositionZero = m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        leftBackPositionZero = m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        rightFrontPositionZero = m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        rightBackPositionZero = m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        System.out.println(">> ENCODERS ZEROED! READY FOR ACTION!");
    }

    /**
     * Convert motor speeds to meters per second
     */
    public double getLeftSpeed() {
        return (speedToMeters(getLeftFrontSpeed()) + speedToMeters(getLeftBackSpeed())) / 2;
    }
    
    public double getRightSpeed() {
        return (speedToMeters(getRightFrontSpeed()) + speedToMeters(getRightBackSpeed())) / 2;
    }
    
    public double getAverageEncoderSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2;
    }

    /**
     * Set wheel speeds using the PID controller and feedforward
     * 
     * @param speeds The desired wheel speeds
     */
    public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedforward = kFeedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = kFeedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput =
            leftPIDController.calculate(getLeftSpeed(), speeds.leftMetersPerSecond);
        final double rightOutput =
            rightPIDController.calculate(getRightSpeed(), speeds.rightMetersPerSecond);

        m_leftMotors.setVoltage(leftOutput + leftFeedforward);
        m_rightMotors.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Convert position to meters
     * 
     * @param position Position in rotations
     * @return Position in meters
     */
    public double positionToMeters(double position) {
        return position * Math.PI * Constants.WHEEL_DIAMETER;
    }
    
    /**
     * Convert speed to meters per second
     * 
     * @param speed Speed in RPM
     * @return Speed in meters per second
     */
    public double speedToMeters(double speed) {
        return speed / 60 * Math.PI * Constants.WHEEL_DIAMETER;
    }
    
    /**
     * Check if robot is moving too fast (safety check)
     * 
     * @return true if speed exceeds safe limit
     */
    public boolean isMovingTooFast() {
        return Math.abs(getAverageEncoderSpeed()) > Constants.MAX_SAFE_SPEED;
    }
    
    /**
     * Set maximum output for safety
     * 
     * @param maxOutput Maximum output (0-1)
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
    
    /**
     * Enable turbo mode - MAXIMUM POWER!
     */
    public void enableTurboMode() {
        setMaxOutput(Constants.DRIVE_TURBO_SPEED);
        SmartDashboard.putBoolean("Turbo Mode", true);
        System.out.println(">>>>>> TURBO MODE ACTIVATED! <<<<<<");
    }
    
    /**
     * Enable precision mode for fine control
     */
    public void enablePrecisionMode() {
        setMaxOutput(Constants.DRIVE_PRECISION_SPEED);
        SmartDashboard.putBoolean("Precision Mode", true);
        System.out.println(">> Precision Mode Engaged");
    }
    
    /**
     * Return to normal driving mode
     */
    public void disableDriveModes() {
        setMaxOutput(Constants.DRIVE_NORMAL_SPEED);
        SmartDashboard.putBoolean("Turbo Mode", false);
        SmartDashboard.putBoolean("Precision Mode", false);
    }

    /**
     * Cartesian drive method for tank-drive hardware
     * 
     * @param xSpeed Speed along X axis (left/right)
     * @param ySpeed Speed along Y axis (forward/backward)
     * @param zRotation Rotation rate around Z axis
     */
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        // For tank drive, we prioritize based on input magnitude
        
        // If we're mostly trying to drive forward/backward
        if (Math.abs(ySpeed) > Math.abs(xSpeed) && Math.abs(ySpeed) > Math.abs(zRotation)) {
            arcadeDrive(ySpeed, 0);
        }
        // If we're mostly trying to rotate
        else if (Math.abs(zRotation) > Math.abs(xSpeed)) {
            arcadeDrive(0, zRotation);
        }
        // If we're mostly trying to strafe (simulated on tank drive)
        else if (Math.abs(xSpeed) > 0.1) {
            // Simulate strafing with differential power
            tankDrive(xSpeed, -xSpeed);
        }
        // If all inputs are very small, just stop
        else {
            stop();
        }
    }
}
