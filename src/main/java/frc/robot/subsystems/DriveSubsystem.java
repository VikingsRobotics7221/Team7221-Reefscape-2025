/*
 * ================================================================
 *  _____   ___ _____ _____   _______  ___  ___  _   _ 
 * |  __ \ |_ _|  _  | ____|  \_   _/ |   \/   || | | |
 * | |  | | | || |_) | |__      | |   | |\  /| || | | |
 * | |  | | | ||  _ <|  __|     | |   | | \/ | || | | |
 * | |__| |_| || |_) | |____   _| |_  | |    | || |_| |
 * |_____/|___|_____/|______| |_____| |_|    |_| \___/ 
 * ================ REEFSCAPE 2025 =========================
 * 
 * TEAM 7221 - THE VIKINGS - DRIVE SUBSYSTEM
 * 
 * This is the HEART of our robot - our 16:1 gear ratio tank drive
 * system that gives us the pushing power to DOMINATE the field!
 * 
 * I spent weeks tuning this code for perfect control and responsiveness.
 * The 16:1 ratio gives us INSANE torque, but only if properly controlled.
 * 
 * Last Updated: March 2025
 * Coded by paysean - Viking Code Warrior
 */

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
import edu.wpi.first.math.MathUtil;

/**
 * DriveSubsystem - The core movement system of our robot!
 * 
 * This class controls our 6-wheel tank drive with a 16:1 gear ratio
 * for MAXIMUM PUSHING POWER! We use arcade-style control for easy
 * driving, with multiple drive modes for different situations.
 * 
 * Features:
 * - Precision 16:1 ratio NEO motor control
 * - Smooth acceleration via slew rate limiting
 * - Multiple drive modes (turbo, normal, precision)
 * - Position tracking via encoder odometry
 * - Auto-correction for drivetrain imbalance
 * - Thermal and current protection
 * - Performance analytics
 */
public class DriveSubsystem extends SubsystemBase {
    
    //------------------------------------------
    // MOTOR CONTROLLERS - THE POWER PLANT
    //------------------------------------------
    private final SparkMax m_leftFrontMotor; // NEO with 16:1 gearing
    private final SparkMax m_rightFrontMotor; // NEO with 16:1 gearing
    private final SparkMax m_leftBackMotor; // NEO with 16:1 gearing
    private final SparkMax m_rightBackMotor; // NEO with 16:1 gearing

    // Motor controller groups for tank drive
    private final MotorControllerGroup m_leftMotors;
    private final MotorControllerGroup m_rightMotors;

    // Differential drive controller for arcade control
    private final DifferentialDrive m_drive;

    //------------------------------------------
    // MOTION CONTROL - SMOOTH ACCELERATION
    //------------------------------------------
    // These slew rate limiters prevent jerky acceleration and protect our drivetrain
    private final SlewRateLimiter m_throttleFilter = new SlewRateLimiter(Constants.THROTTLE_SLEW_RATE);
    private final SlewRateLimiter m_turnFilter = new SlewRateLimiter(Constants.TURN_SLEW_RATE);

    //------------------------------------------
    // DRIVE STATE TRACKING
    //------------------------------------------
    // Encoder zero points
    private double m_leftFrontZero = 0.0;
    private double m_rightFrontZero = 0.0;
    private double m_leftBackZero = 0.0;
    private double m_rightBackZero = 0.0;
    
    // Simulated gyro for position tracking
    private double m_simulatedAngle = 0.0;
    private long m_lastUpdateTime = 0;
    
    // Performance tracking
    private double m_maxCurrent = 0.0;
    private double m_maxTemperature = 0.0;
    private double m_lastLoopTime = 0.0;
    
    // Drive mode tracking
    private boolean m_turboMode = false;
    private boolean m_precisionMode = false;
    private int m_driveMode = 0; // 0=normal, 1=turbo, 2=precision
    
    // Distance tracking 
    private double m_totalDistance = 0.0;
    private double m_lastLeftPosition = 0.0;
    private double m_lastRightPosition = 0.0;

    //------------------------------------------
    // KINEMATICS & ODOMETRY - POSITION TRACKING
    //------------------------------------------
    private static final double TRACK_WIDTH = Constants.TRACK_WIDTH;
    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    private final DifferentialDriveOdometry m_odometry;
    
    //------------------------------------------
    // CONTROL SYSTEMS - MOTION PROFILING
    //------------------------------------------
    // Feedforward and PID controllers for velocity control
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.18, 2.85, 0.48);
    private final PIDController m_leftPIDController = new PIDController(Constants.kP_FRONT_LEFT_VELOCITY, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(Constants.kP_FRONT_RIGHT_VELOCITY, 0, 0);
    
    // Motion constraints for path following
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        Constants.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
        Constants.kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    /**
     * Creates a new DriveSubsystem - the heart of our robot's movement!
     * This is where the MAGIC happens - our 16:1 gear ratio tank drive.
     */
    public DriveSubsystem() {
        System.out.println("\n" +
                          "======================================================\n" +
                          ">> INITIALIZING 16:1 BEAST-MODE DRIVETRAIN!          \n" +
                          ">> MAXIMUM TORQUE CONFIGURATION LOADING...            \n" +
                          ">> PREPARE FOR DOMINATION!                           \n" +
                          "======================================================");
        
        // Initialize the SparkMAX controllers for each NEO motor
        m_leftFrontMotor = new SparkMax(Constants.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_rightFrontMotor = new SparkMax(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_leftBackMotor = new SparkMax(Constants.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_rightBackMotor = new SparkMax(Constants.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);

        // Configure each motor with optimal settings for 16:1 gear ratio
        configureSparkMAX(m_leftFrontMotor, Constants.REVERSE_LEFT_FRONT_MOTOR);
        configureSparkMAX(m_leftBackMotor, Constants.REVERSE_LEFT_BACK_MOTOR);
        configureSparkMAX(m_rightFrontMotor, Constants.REVERSE_RIGHT_FRONT_MOTOR);
        configureSparkMAX(m_rightBackMotor, Constants.REVERSE_RIGHT_BACK_MOTOR);

        // Create motor groups for left and right side control
        m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
        m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

        // Create the differential drive controller for arcade drive
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        m_drive.setDeadband(Constants.JOYSTICK_DEADBAND);

        // Initialize the odometry system for position tracking (no gyro)
        m_odometry = new DifferentialDriveOdometry(
            new Rotation2d(),  // No real gyro, using simulated angle
            0, 0);            // Starting position

        // Reset encoders to establish our zero point
        resetEncoders();

        // Initialize our simulated gyro timestamp
        m_lastUpdateTime = System.currentTimeMillis();
        
        // Display status information
        System.out.println(">> 16:1 DRIVETRAIN ACTIVATED - MAXIMUM TORQUE READY!");
        System.out.println(">> DRIVE MODES AVAILABLE:");
        System.out.println(">>   - NORMAL MODE: " + Constants.DRIVE_NORMAL_SPEED * 100 + "% power");
        System.out.println(">>   - TURBO MODE: " + Constants.DRIVE_TURBO_SPEED * 100 + "% power");
        System.out.println(">>   - PRECISION MODE: " + Constants.DRIVE_PRECISION_SPEED * 100 + "% power");
    }

    /**
     * Configures a SparkMAX motor controller with optimal settings for 16:1 ratio
     * 
     * @param motor Motor controller to configure
     * @param inverted Whether to invert the motor direction
     */
    private void configureSparkMAX(SparkMax motor, boolean inverted) {
        // Create optimal configuration for 16:1 gearing
        SparkMaxConfig config = new SparkMaxConfig();
        
        // Brake mode is critical for 16:1 ratio to prevent coasting!
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        
        // Apply our configuration to the motor
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Set current limit to protect motors with high-torque 16:1 ratio
        try {
            // 30 amps is sufficient protection for NEO motors
            motor.setSmartCurrentLimit(30);
            System.out.println(">> Current limit set to 30A for motor protection");
        } catch (Exception e) {
            System.out.println("!! WARNING: Failed to set current limit! Watch your battery level!");
            e.printStackTrace();
        }
    }

    /**
     * Emergency stop all drive motors - THE KILL SWITCH!
     */
    public void stop() {
        m_drive.stopMotor();
        System.out.println(">> EMERGENCY STOP ACTIVATED - ALL DRIVE MOTORS HALTED!");
    }

    /**
     * Reset simulated gyro angle to zero
     */
    public void zeroGyro() {
        m_simulatedAngle = 0.0;
        System.out.println(">> SIMULATED GYRO ZEROED!");
    }
    
    /**
     * Get the simulated yaw angle (rotation around vertical axis)
     * 
     * @return Simulated yaw angle in degrees
     */
    public double getYaw() {
        return m_simulatedAngle;
    }
    
    /**
     * Get the simulated pitch angle (rotation around side-to-side axis)
     * Note: We don't have a real gyro, so this always returns 0
     * 
     * @return Simulated pitch angle in degrees (always 0)
     */
    public double getPitch() {
        return 0.0; // No pitch detection without real gyro
    }
    
    /**
     * Get the simulated roll angle (rotation around front-to-back axis)
     * Note: We don't have a real gyro, so this always returns 0
     * 
     * @return Simulated roll angle in degrees (always 0)
     */
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
     * Get estimated turn rate based on differential wheel speeds
     * 
     * @return Estimated turn rate in degrees per second
     */
    public double getTurnRate() {
        // Estimate based on differential wheel speeds
        double leftSpeed = getLeftSpeed();
        double rightSpeed = getRightSpeed();
        double turnRate = Math.toDegrees((rightSpeed - leftSpeed) / TRACK_WIDTH);
        return turnRate;
    }

    /**
     * Get the rotation as a Rotation2d object for odometry
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
        m_odometry.resetPosition(this.getRotation2d(), getLeftPosition(), getRightPosition(), pose);
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
     * Get the drive kinematics object
     * 
     * @return Drive kinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }
    
    /**
     * Get the motion constraints
     * 
     * @return Motion constraints
     */
    public TrapezoidProfile.Constraints getConstraints() {
        return m_constraints;
    }

    @Override
    public void periodic() {
        // Record the start time of this loop for performance monitoring
        long loopStartTime = System.currentTimeMillis();
        
        // Update the odometry with current encoder readings
        m_odometry.update(this.getRotation2d(), getLeftPosition(), getRightPosition());

        // Update our simulated gyro based on differential wheel movement
        updateSimulatedGyro();

        // Track total distance traveled
        updateDistanceTraveled();
        
        // Monitor motor performance
        monitorMotorPerformance();

        // Log drive information to SmartDashboard
        updateDashboard();
        
        // Calculate loop execution time for performance monitoring
        double loopTime = (System.currentTimeMillis() - loopStartTime) / 1000.0;
        m_lastLoopTime = loopTime;
        
        // Warn if loop is taking too long
        if (loopTime > Constants.LOOP_TIME_WARNING) {
            System.out.println(">> WARNING: Drive loop taking " + loopTime + "s (target: " + 
                              Constants.TARGET_LOOP_TIME + "s)");
        }
    }
    
    /**
     * Update the dashboard with current drive status
     */
    private void updateDashboard() {
        // Position information
        SmartDashboard.putNumber("Left Position", getLeftPosition());
        SmartDashboard.putNumber("Right Position", getRightPosition());
        SmartDashboard.putNumber("Simulated Angle", m_simulatedAngle);
        SmartDashboard.putNumber("Total Distance", m_totalDistance);
        
        // Speed information
        SmartDashboard.putNumber("Left Speed", getLeftSpeed());
        SmartDashboard.putNumber("Right Speed", getRightSpeed());
        
        // Performance information
        SmartDashboard.putNumber("Left Front Current", m_leftFrontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Right Front Current", m_rightFrontMotor.getOutputCurrent());
        SmartDashboard.putNumber("Max Current", m_maxCurrent);
        SmartDashboard.putNumber("Max Temperature", m_maxTemperature);
        SmartDashboard.putNumber("Loop Time", m_lastLoopTime * 1000); // ms
        
        // Drive mode information
        SmartDashboard.putBoolean("Turbo Mode", m_turboMode);
        SmartDashboard.putBoolean("Precision Mode", m_precisionMode);
        SmartDashboard.putNumber("Drive Mode", m_driveMode);
    }
    
    /**
     * Monitor motor current and temperature
     */
    private void monitorMotorPerformance() {
        // Check all motor currents
        double lf = m_leftFrontMotor.getOutputCurrent();
        double rf = m_rightFrontMotor.getOutputCurrent();
        double lb = m_leftBackMotor.getOutputCurrent();
        double rb = m_rightBackMotor.getOutputCurrent();
        
        // Track maximum current for diagnostics
        double maxCurrent = Math.max(Math.max(lf, rf), Math.max(lb, rb));
        if (maxCurrent > m_maxCurrent) {
            m_maxCurrent = maxCurrent;
        }
        
        // Check all motor temperatures (if available)
        try {
            double lft = m_leftFrontMotor.getMotorTemperature();
            double rft = m_rightFrontMotor.getMotorTemperature();
            double lbt = m_leftBackMotor.getMotorTemperature();
            double rbt = m_rightBackMotor.getMotorTemperature();
            
            // Track maximum temperature
            double maxTemp = Math.max(Math.max(lft, rft), Math.max(lbt, rbt));
            if (maxTemp > m_maxTemperature) {
                m_maxTemperature = maxTemp;
            }
            
            // Warn if any motors are getting too hot
            if (maxTemp > Constants.MOTOR_TEMPERATURE_WARNING) {
                System.out.println(">> WARNING: Motor temperature high: " + maxTemp + "°C");
            }
            
            // Emergency if any motors are critically hot
            if (maxTemp > Constants.MOTOR_TEMPERATURE_CRITICAL) {
                System.out.println(">> CRITICAL: Motor temperature extreme: " + maxTemp + "°C");
                setMaxOutput(0.5); // Reduce power to prevent damage
            }
        } catch (Exception e) {
            // Temperature reading not available - this is normal for some motor controllers
        }
    }
    
    /**
     * Track the total distance traveled by the robot
     */
    private void updateDistanceTraveled() {
        double leftPos = getLeftPosition();
        double rightPos = getRightPosition();
        
        // Calculate distance traveled since last update
        double leftDelta = Math.abs(leftPos - m_lastLeftPosition);
        double rightDelta = Math.abs(rightPos - m_lastRightPosition);
        double distanceDelta = (leftDelta + rightDelta) / 2.0;
        
        // Add to total distance (convert from rotations to meters)
        m_totalDistance += distanceDelta * Constants.WHEEL_CIRCUMFERENCE;
        
        // Update last positions
        m_lastLeftPosition = leftPos;
        m_lastRightPosition = rightPos;
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
     * Arcade drive method for our tank-drive hardware.
     * This is the PRIMARY driving method that makes our 16:1 ratio robot drive smoothly!
     * 
     * @param xSpeed Forward/backward speed (-1.0..1.0). Forward is positive.
     * @param zRotation Rotation rate around Z axis (-1.0..1.0). Counterclockwise is positive.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Apply deadband and cap inputs at 1.0
        xSpeed = MathUtil.applyDeadband(xSpeed, Constants.JOYSTICK_DEADBAND);
        zRotation = MathUtil.applyDeadband(zRotation, Constants.JOYSTICK_DEADBAND);
        
        // Apply filtering for smooth acceleration - THIS IS CRITICAL FOR 16:1 RATIO!
        xSpeed = m_throttleFilter.calculate(xSpeed);
        zRotation = m_turnFilter.calculate(zRotation);
        
        // Square inputs for better low-speed control but keep the sign
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);
        
        // Send the commands to the drivetrain
        m_drive.arcadeDrive(xSpeed, zRotation, false); // Already squared the inputs
    }

    /**
     * Tank drive method for direct control of left and right sides.
     * Less commonly used, but useful for some autonomous routines.
     * 
     * @param leftSpeed Left side speed (-1.0..1.0)
     * @param rightSpeed Right side speed (-1.0..1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        // Apply deadband
        leftSpeed = MathUtil.applyDeadband(leftSpeed, Constants.JOYSTICK_DEADBAND);
        rightSpeed = MathUtil.applyDeadband(rightSpeed, Constants.JOYSTICK_DEADBAND);
        
        // Apply filtering for smooth acceleration
        leftSpeed = m_throttleFilter.calculate(leftSpeed);
        rightSpeed = m_throttleFilter.calculate(rightSpeed);
        
        // Apply tank drive with squared inputs for better control
        m_drive.tankDrive(leftSpeed, rightSpeed, true);
    }

    /**
     * Get left front encoder position in rotations
     * 
     * @return Position in rotations
     */
    public double getLeftFrontPosition() {
        return (m_leftFrontMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO - m_leftFrontZero);
    }
    
    /**
     * Get right front encoder position in rotations
     * 
     * @return Position in rotations
     */
    public double getRightFrontPosition() {
        return (m_rightFrontMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO - m_rightFrontZero);
    }
    
    /**
     * Get left back encoder position in rotations
     * 
     * @return Position in rotations
     */
    public double getLeftBackPosition() {
        return (m_leftBackMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO - m_leftBackZero);
    }
    
    /**
     * Get right back encoder position in rotations
     * 
     * @return Position in rotations
     */
    public double getRightBackPosition() {
        return (m_rightBackMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO - m_rightBackZero);
    }
    
    /**
     * Get motor speeds in RPM
     * Adjusted for 16:1 gearing
     * 
     * @return Motor speed in RPM
     */
    public double getLeftFrontSpeed() {
        return (m_leftFrontMotor.getEncoder().getVelocity() / Constants.DRIVE_GEAR_RATIO);
    }
    
    /**
     * Get right front motor speed
     * 
     * @return Speed in RPM
     */
    public double getRightFrontSpeed() {
        return (m_rightFrontMotor.getEncoder().getVelocity() / Constants.DRIVE_GEAR_RATIO);
    }
    
    /**
     * Get left back motor speed
     * 
     * @return Speed in RPM
     */
    public double getLeftBackSpeed() {
        return (m_leftBackMotor.getEncoder().getVelocity() / Constants.DRIVE_GEAR_RATIO);
    }
    
    /**
     * Get right back motor speed
     * 
     * @return Speed in RPM
     */
    public double getRightBackSpeed() {
        return (m_rightBackMotor.getEncoder().getVelocity() / Constants.DRIVE_GEAR_RATIO);
    }

    /**
     * Get average left side position for odometry
     * 
     * @return Left position in rotations
     */
    public double getLeftPosition() {
        return (getLeftFrontPosition() + getLeftBackPosition()) / 2.0;
    }

    /**
     * Get average right side position for odometry
     * 
     * @return Right position in rotations
     */
    public double getRightPosition() {
        return (getRightFrontPosition() + getRightBackPosition()) / 2.0;
    }

    /**
     * Reset all encoder values to establish new zero point
     * CRITICAL for autonomous routines!
     */
    public void resetEncoders() {
        m_leftFrontZero = m_leftFrontMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO;
        m_leftBackZero = m_leftBackMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO;
        m_rightFrontZero = m_rightFrontMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO;
        m_rightBackZero = m_rightBackMotor.getEncoder().getPosition() / Constants.DRIVE_GEAR_RATIO;
        
        // Also reset our distance tracking
        m_totalDistance = 0.0;
        m_lastLeftPosition = 0.0;
        m_lastRightPosition = 0.0;
        
        System.out.println(">> ENCODERS ZEROED! POSITION TRACKING RESET!");
    }

    /**
     * Convert motor speeds to meters per second
     * 
     * @return Speed in meters per second
     */
    public double getLeftSpeed() {
        return (speedToMeters(getLeftFrontSpeed()) + speedToMeters(getLeftBackSpeed())) / 2;
    }
    
    /**
     * Get right side speed in meters per second
     * 
     * @return Speed in meters per second
     */
    public double getRightSpeed() {
        return (speedToMeters(getRightFrontSpeed()) + speedToMeters(getRightBackSpeed())) / 2;
    }
    
    /**
     * Get average robot speed
     * 
     * @return Average speed in meters per second
     */
    public double getAverageSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2;
    }

    /**
     * Set wheel speeds using PID controller and feedforward
     * For trajectory following
     * 
     * @param speeds Desired wheel speeds
     */
    public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Calculate feedforward values
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        // Calculate PID corrections
        final double leftOutput =
            m_leftPIDController.calculate(getLeftSpeed(), speeds.leftMetersPerSecond);
        final double rightOutput =
            m_rightPIDController.calculate(getRightSpeed(), speeds.rightMetersPerSecond);

        // Apply combined outputs as voltage
        m_leftMotors.setVoltage(leftOutput + leftFeedforward);
        m_rightMotors.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Convert position in rotations to meters
     * 
     * @param position Position in rotations
     * @return Position in meters
     */
    public double positionToMeters(double position) {
        return position * Constants.WHEEL_CIRCUMFERENCE;
    }
    
    /**
     * Convert speed in RPM to meters per second
     * 
     * @param speed Speed in RPM
     * @return Speed in meters per second
     */
    public double speedToMeters(double speed) {
        return speed / 60 * Constants.WHEEL_CIRCUMFERENCE;
    }
    
    /**
     * Check if robot is moving too fast (safety check)
     * 
     * @return true if speed exceeds safe limit
     */
    public boolean isMovingTooFast() {
        return Math.abs(getAverageSpeed()) > Constants.MAX_SAFE_SPEED;
    }
    
    /**
     * Set maximum drive output for different drive modes
     * 
     * @param maxOutput Maximum output (0-1)
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
    
    /**
     * Enable turbo mode - MAXIMUM POWER!
     * Use with caution - this is full power with 16:1 ratio!
     */
    public void enableTurboMode() {
        setMaxOutput(Constants.DRIVE_TURBO_SPEED);
        m_turboMode = true;
        m_precisionMode = false;
        m_driveMode = 1; // Turbo
        SmartDashboard.putBoolean("Turbo Mode", true);
        System.out.println(">> TURBO MODE ACTIVATED! MAXIMUM POWER!");
    }
    
    /**
     * Enable precision mode for fine control
     * Great for lining up with game pieces or the barge
     */
    public void enablePrecisionMode() {
        setMaxOutput(Constants.DRIVE_PRECISION_SPEED);
        m_precisionMode = true;
        m_turboMode = false;
        m_driveMode = 2; // Precision
        SmartDashboard.putBoolean("Precision Mode", true);
        System.out.println(">> PRECISION MODE ACTIVATED! Fine control enabled.");
    }
    
    /**
     * Return to normal driving mode
     * Balanced performance and control
     */
    public void disableDriveModes() {
        setMaxOutput(Constants.DRIVE_NORMAL_SPEED);
        m_turboMode = false;
        m_precisionMode = false;
        m_driveMode = 0; // Normal
        SmartDashboard.putBoolean("Turbo Mode", false);
        SmartDashboard.putBoolean("Precision Mode", false);
    }

    /**
     * Simulated "Cartesian" drive for tank drive
     * This lets us pretend we have mecanum wheels even with tank drive
     * 
     * @param xSpeed Forward speed (positive = forward)
     * @param ySpeed Strafe speed (not really possible with tank drive)
     * @param zRotation Rotation speed
     */
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        // Forward/backward motion takes priority
        if (Math.abs(xSpeed) > Math.abs(ySpeed) && Math.abs(xSpeed) > Math.abs(zRotation)) {
            arcadeDrive(xSpeed, 0);
        }
        // Rotation takes second priority
        else if (Math.abs(zRotation) > Math.abs(ySpeed)) {
            arcadeDrive(0, zRotation);
        }
        // Attempt to "strafe" by doing a tank turn (not very effective but it's something)
        else if (Math.abs(ySpeed) > 0.1) {
            // Simulate strafing by spinning the tracks in opposite directions
            tankDrive(ySpeed, -ySpeed);
        }
        // If all inputs are very small, just stop
        else {
            stop();
        }
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
     * Get the current drive mode as a string
     * 
     * @return Drive mode name
     */
    public String getDriveModeName() {
        if (m_turboMode) return "TURBO";
        if (m_precisionMode) return "PRECISION";
        return "NORMAL";
    }
    
    /**
     * Get the maximum motor current observed
     * 
     * @return Maximum current in amps
     */
    public double getMaxCurrent() {
        return m_maxCurrent;
    }
    
    /**
     * Get the maximum motor temperature observed
     * 
     * @return Maximum temperature in degrees C
     */
    public double getMaxTemperature() {
        return m_maxTemperature;
    }
    
    /**
     * Reset the performance tracking variables
     */
    public void resetPerformanceTracking() {
        m_maxCurrent = 0.0;
        m_maxTemperature = 0.0;
        System.out.println(">> Performance tracking reset!");
    }
}
