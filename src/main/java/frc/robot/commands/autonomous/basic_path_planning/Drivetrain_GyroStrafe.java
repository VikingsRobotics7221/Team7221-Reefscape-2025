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
 * Drivetrain - TEAM 7221 TANK DRIVE SUBSYSTEM
 * 
 * The heart of our robot - tank drive with arcade control for MAXIMUM MANEUVERABILITY!
 * Handles motor control, encoder tracking, and driver assists.
 * 
 * coded by paysean
 */
public class DriveSubsystem extends SubsystemBase {
  
    // Drivetrain Motor Controllers
    private static SparkMax m_leftFrontMotor; // NEO motor
    private static SparkMax m_rightFrontMotor; // NEO motor
    private static SparkMax m_leftBackMotor; // NEO motor
    private static SparkMax m_rightBackMotor; // NEO motor

    // Motor controller groups for left and right sides
    private MotorControllerGroup m_leftMotors;
    private MotorControllerGroup m_rightMotors;

    // Differential drive for tank drive with arcade control
    private DifferentialDrive m_drive;

    // Slew rate limiters for SMOOTH acceleration (no jerky movements!)
    SlewRateLimiter throttleFilter;
    SlewRateLimiter turnFilter;

    // Drive constants
    private double DRIVE_GEAR_RATIO = Constants.DRIVE_GEAR_RATIO;

    // Variables for encoder tracking
    double leftFrontPositionZero, rightFrontPositionZero, leftBackPositionZero, rightBackPositionZero = 0.0;
    
    // Variables for turn simulation (replacing gyro)
    private double m_simulatedAngle = 0.0;
    private long m_lastUpdateTime = 0;

    private static final double TRACK_WIDTH = Constants.TRACK_WIDTH;

    private static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(0.17472, 2.7572, 0.45109);
    private static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
            Constants.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
            Constants.kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
    
    private final PIDController leftPIDController = new PIDController(Constants.kP_FRONT_LEFT_VELOCITY, 0, 0);
    private final PIDController rightPIDController = new PIDController(Constants.kP_FRONT_RIGHT_VELOCITY, 0, 0);

    private static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    private static DifferentialDriveOdometry odometry;
  
    /** 
     * Creates THE BEAST - our drivetrain subsystem!
     */
    public DriveSubsystem() {
        System.out.println(">> INITIALIZING DRIVETRAIN - TEAM 7221 POWER SYSTEM <<");
        
        // Instantiate the Drivetrain motor controllers
        m_leftFrontMotor = new SparkMax(Constants.LEFT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_rightFrontMotor = new SparkMax(Constants.RIGHT_FRONT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_leftBackMotor = new SparkMax(Constants.LEFT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);
        m_rightBackMotor = new SparkMax(Constants.RIGHT_REAR_DRIVE_MOTOR_ID, MotorType.kBrushless);

        // Configure the Spark MAX motor controllers
        configureSparkMAX(m_leftFrontMotor, Constants.REVERSE_LEFT_FRONT_MOTOR);
        configureSparkMAX(m_leftBackMotor, Constants.REVERSE_LEFT_BACK_MOTOR);
        configureSparkMAX(m_rightBackMotor, Constants.REVERSE_RIGHT_BACK_MOTOR);
        configureSparkMAX(m_rightFrontMotor, Constants.REVERSE_RIGHT_FRONT_MOTOR);

        // Create motor controller groups for tank drive
        m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);
        m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

        // Create differential drive (tank drive)
        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
        
        // Set dead band to reduce small unwanted movements from joystick noise
        m_drive.setDeadband(0.05);

        // Create odometry without gyro - using encoder-only odometry
        odometry = new DifferentialDriveOdometry(
            new Rotation2d(),  // No gyro, so use dummy rotation
            0, 0);

        resetEncoders(); // Zero the drive encoders

        // Create slew rate limiters for smooth acceleration
        throttleFilter = new SlewRateLimiter(3.0); // Units per second
        turnFilter = new SlewRateLimiter(3.0);     // Units per second
        
        // Initialize the timestamp for gyro simulation
        m_lastUpdateTime = System.currentTimeMillis();

        System.out.println(">> DRIVETRAIN ONLINE - WHEELS READY TO ROLL!!! <<");
    }

    private void configureSparkMAX(SparkMax max, boolean reverse) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(reverse).idleMode(IdleMode.kBrake);
        max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stop() {
        m_drive.stopMotor();
    }

    // GYRO REPLACEMENT METHODS - These simulate a gyro using encoder data
    public void zeroGyro() {
        // Reset our simulated angle
        m_simulatedAngle = 0.0;
    }
    
    public double getYaw() {
        return 0.0; // No real gyro, return zero
    }
    
    public double getPitch() {
        return 0.0; // No real gyro, return zero
    }
    
    public double getRoll() {
        return 0.0; // No real gyro, return zero
    }
    
    // This simulates a gyro angle based on differential encoder readings
    public double getGyroAngle() {
        // Update simulated angle based on encoder differential
        return m_simulatedAngle;
    }
    
    public double getTurnRate() {
        return 0.0; // No real gyro, return zero
    }

    /** Odometry Methods *******************************************************/
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(m_simulatedAngle);
    }
    
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(this.getRotation2d(), getLeftPosition(), getRightPosition(), pose);
    }
    
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    
    public DifferentialDriveKinematics getkDriveKinematics() {
        return kDriveKinematics;    
    }
    
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
    }
    
    // This method estimates rotation based on differential wheel speeds
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
     * Arcade drive method for tank drive hardware.
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        // Apply filtering for smooth acceleration
        xSpeed = throttleFilter.calculate(xSpeed);
        zRotation = turnFilter.calculate(zRotation);
        
        m_drive.arcadeDrive(xSpeed, zRotation, true); // Square inputs for finer control
    }

    /**
     * Tank drive method for direct control of left and right sides.
     * @param leftSpeed Left side speed
     * @param rightSpeed Right side speed
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        // Apply filtering for smooth acceleration
        leftSpeed = throttleFilter.calculate(leftSpeed);
        rightSpeed = throttleFilter.calculate(rightSpeed);
        
        m_drive.tankDrive(leftSpeed, rightSpeed, true); // Square inputs for finer control
    }

    /** Get the encoder positions or speeds **************************************/
    public double getLeftFrontPosition() { // Position is returned in units of revolutions
        return (m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftFrontPositionZero);
    }
    public double getRightFrontPosition() { // Position is returned in units of revolutions
        return -1 * (m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightFrontPositionZero);
    }
    public double getLeftBackPosition() { // Position is returned in units of revolutions
        return -1 * (m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - leftBackPositionZero);
    }
    public double getRightBackPosition() { // Position is returned in units of revolutions
        return (m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO - rightBackPositionZero);
    }
    public double getLeftFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
        return (m_leftFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }
    public double getRightFrontSpeed() { // Speed is returned in units of RPM (revolutions per minute)
        return -1 * (m_rightFrontMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }
    public double getLeftBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
        return -1 * (m_leftBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }
    public double getRightBackSpeed() { // Speed is returned in units of RPM (revolutions per minute)
        return (m_rightBackMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO);
    }

    // Combined positions for odometry
    public double getLeftPosition() {
        return (getLeftFrontPosition() + getLeftBackPosition()) / 2.0;
    }

    public double getRightPosition() {
        return (getRightFrontPosition() + getRightBackPosition()) / 2.0;
    }

    // Zero the drivetrain encoders
    public void resetEncoders() {
        leftFrontPositionZero = m_leftFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        leftBackPositionZero = m_leftBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        rightFrontPositionZero = m_rightFrontMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
        rightBackPositionZero = m_rightBackMotor.getEncoder().getPosition() / DRIVE_GEAR_RATIO;
    }

    // Speed will be measured in meters/second
    public double getLeftSpeed() {
        return (speedToMeters(getLeftFrontSpeed()) + speedToMeters(getLeftBackSpeed())) / 2;
    }
    public double getRightSpeed() {
        return (speedToMeters(getRightFrontSpeed()) + speedToMeters(getRightBackSpeed())) / 2;
    }
    public double getAverageEncoderSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2;
    }

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

    // Conversion Methods: Convert position & speed to Meters
    public double positionToMeters(double position) {
        return position * Math.PI * Constants.WHEEL_DIAMETER;
    }
    
    public double speedToMeters(double speed) {
        return speed / 60 * Math.PI * Constants.WHEEL_DIAMETER;
    }
    
    // Returns true if the robot is moving too fast (useful for safety checks)
    public boolean isMovingTooFast() {
        return Math.abs(getAverageEncoderSpeed()) > Constants.MAX_SAFE_SPEED;
    }
    
    // Set the maximum output of the drive system for safety
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
    
    // Methods for turbo and precision modes
    public void enableTurboMode() {
        setMaxOutput(Constants.DRIVE_TURBO_SPEED);
        SmartDashboard.putBoolean("Turbo Mode", true);
        System.out.println(">>>>>> TURBO MODE ACTIVATED! <<<<<<");
    }
    
    public void enablePrecisionMode() {
        setMaxOutput(Constants.DRIVE_PRECISION_SPEED);
        SmartDashboard.putBoolean("Precision Mode", true);
        System.out.println(">> Precision Mode Engaged");
    }
    
    public void disableDriveModes() {
        setMaxOutput(Constants.DRIVE_NORMAL_SPEED);
        SmartDashboard.putBoolean("Turbo Mode", false);
        SmartDashboard.putBoolean("Precision Mode", false);
    }

    /**
    * Cartesian drive method for holonomic-like control on a tank drive
    * This allows for "strafing" by simulating it with tank drive
    * 
    * @param xSpeed Speed along X axis (left/right)
    * @param ySpeed Speed along Y axis (forward/backward)
    * @param zRotation Rotation rate around Z axis
    */
    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        // For tank drive, we can't strafe, so we prioritize
        // either forward/backward or rotation depending on inputs
        
        // If we're mostly trying to drive forward/backward
        if (Math.abs(ySpeed) > Math.abs(xSpeed) && Math.abs(ySpeed) > Math.abs(zRotation)) {
            arcadeDrive(ySpeed, 0);
        }
        // If we're mostly trying to rotate
        else if (Math.abs(zRotation) > Math.abs(xSpeed)) {
            arcadeDrive(0, zRotation);
        }
        // If we're mostly trying to strafe (which we can't really do)
        // We'll simulate it by using a tank-turning approach
        else if (Math.abs(xSpeed) > 0.1) {
            // This is a rough approximation
            tankDrive(xSpeed, -xSpeed);
        }
        // If all inputs are very small, just stop
        else {
            stop();
        }
    }
}
