package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorSafetyMonitor;

/**
 * DriveSubsystem handles the robot's main drivetrain functionality.
 * This subsystem controls the robot's movement during both autonomous and teleop periods.
 */
public class DriveSubsystem extends SubsystemBase {
    // Motor controllers for the left and right sides of the drivetrain
    private final CANSparkMax leftLeadMotor;
    private final CANSparkMax leftFollowMotor;
    private final CANSparkMax rightLeadMotor;
    private final CANSparkMax rightFollowMotor;
    
    // Motor encoders
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    // NavX gyroscope for heading and balance information
    private final AHRS gyro;

    // Differential drive controller for simplified drive control
    private final DifferentialDrive differentialDrive;

    // Odometry for tracking robot position
    private final DifferentialDriveOdometry odometry;

    // Motor safety monitor to prevent overheating and damage
    private final MotorSafetyMonitor safetyMonitor;

    /**
     * Creates a new DriveSubsystem.
     * Initializes motors, sensors, and drive controllers.
     */
    public DriveSubsystem() {
        // Initialize drive motors with CAN IDs from Constants
        leftLeadMotor = new CANSparkMax(Constants.DriveConstants.LEFT_LEAD_MOTOR_ID, MotorType.kBrushless);
        leftFollowMotor = new CANSparkMax(Constants.DriveConstants.LEFT_FOLLOW_MOTOR_ID, MotorType.kBrushless);
        rightLeadMotor = new CANSparkMax(Constants.DriveConstants.RIGHT_LEAD_MOTOR_ID, MotorType.kBrushless);
        rightFollowMotor = new CANSparkMax(Constants.DriveConstants.RIGHT_FOLLOW_MOTOR_ID, MotorType.kBrushless);

        // Get encoders from the motors
        leftEncoder = leftLeadMotor.getEncoder();
        rightEncoder = rightLeadMotor.getEncoder();

        // Configure motors
        configureMotors();

        // Initialize gyro on the standard SPI port
        gyro = new AHRS(SPI.Port.kMXP);
        resetGyro();

        // Set up differential drive controller with lead motors
        differentialDrive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
        differentialDrive.setDeadband(Constants.DriveConstants.DEADBAND);
        
        // Initialize odometry with initial position at origin
        odometry = new DifferentialDriveOdometry(
            getRotation2d(),
            0.0,  // Left distance
            0.0   // Right distance
        );

        // Initialize motor safety monitor with all drive motors
        safetyMonitor = new MotorSafetyMonitor(
            new CANSparkMax[] {leftLeadMotor, leftFollowMotor, rightLeadMotor, rightFollowMotor},
            Constants.DriveConstants.MOTOR_TEMPERATURE_THRESHOLD
        );

        // Register with safety monitor
        safetyMonitor.registerSubsystem(this);
    }

    /**
     * Configure all drive motors with appropriate settings.
     */
    private void configureMotors() {
        // Reset motors to factory defaults
        leftLeadMotor.restoreFactoryDefaults();
        leftFollowMotor.restoreFactoryDefaults();
        rightLeadMotor.restoreFactoryDefaults();
        rightFollowMotor.restoreFactoryDefaults();

        // Set motor directions (right side is typically inverted)
        leftLeadMotor.setInverted(Constants.DriveConstants.LEFT_MOTORS_INVERTED);
        rightLeadMotor.setInverted(Constants.DriveConstants.RIGHT_MOTORS_INVERTED);

        // Configure the follower motors
        leftFollowMotor.follow(leftLeadMotor, Constants.DriveConstants.LEFT_MOTORS_INVERTED);
        rightFollowMotor.follow(rightLeadMotor, Constants.DriveConstants.RIGHT_MOTORS_INVERTED);

        // Configure encoder conversion factors
        leftEncoder.setPositionConversionFactor(Constants.DriveConstants.POSITION_TO_METERS);
        rightEncoder.setPositionConversionFactor(Constants.DriveConstants.POSITION_TO_METERS);
        leftEncoder.setVelocityConversionFactor(Constants.DriveConstants.VELOCITY_TO_METERS_PER_SECOND);
        rightEncoder.setVelocityConversionFactor(Constants.DriveConstants.VELOCITY_TO_METERS_PER_SECOND);

        // Set current limits to protect motors
        leftLeadMotor.setSmartCurrentLimit(Constants.DriveConstants.CURRENT_LIMIT);
        leftFollowMotor.setSmartCurrentLimit(Constants.DriveConstants.CURRENT_LIMIT);
        rightLeadMotor.setSmartCurrentLimit(Constants.DriveConstants.CURRENT_LIMIT);
        rightFollowMotor.setSmartCurrentLimit(Constants.DriveConstants.CURRENT_LIMIT);

        // Set idle mode (brake or coast)
        setBrakeMode(true);

        // Set ramp rate to prevent sudden acceleration
        leftLeadMotor.setOpenLoopRampRate(Constants.DriveConstants.RAMP_RATE);
        rightLeadMotor.setOpenLoopRampRate(Constants.DriveConstants.RAMP_RATE);
        leftFollowMotor.setOpenLoopRampRate(Constants.DriveConstants.RAMP_RATE);
        rightFollowMotor.setOpenLoopRampRate(Constants.DriveConstants.RAMP_RATE);

        // Burn settings into flash memory
        leftLeadMotor.burnFlash();
        leftFollowMotor.burnFlash();
        rightLeadMotor.burnFlash();
        rightFollowMotor.burnFlash();
    }
    
    /**
     * Called periodically by the CommandScheduler.
     * Updates odometry and sends data to SmartDashboard.
     */
    @Override
    public void periodic() {
        // Update the robot's position
        updateOdometry();
        
        // Post drive information to SmartDashboard for debugging
        updateSmartDashboard();
        
        // Check motor safety
        safetyMonitor.checkSafety();
    }

    /**
     * Updates the robot's odometry with the latest sensor readings.
     */
    private void updateOdometry() {
        odometry.update(
            getRotation2d(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );
    }

    /**
     * Updates SmartDashboard with drive-related information.
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Left Drive Distance (m)", getLeftDistanceMeters());
        SmartDashboard.putNumber("Right Drive Distance (m)", getRightDistanceMeters());
        
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Robot X (m)", pose.getX());
        SmartDashboard.putNumber("Robot Y (m)", pose.getY());
        SmartDashboard.putNumber("Robot Rotation (deg)", pose.getRotation().getDegrees());
        
        SmartDashboard.putNumber("Left Motor Temp", leftLeadMotor.getMotorTemperature());
        SmartDashboard.putNumber("Right Motor Temp", rightLeadMotor.getMotorTemperature());
    }

    /**
     * Drive using tank drive controls (separate left/right control).
     * Used by teleop and some autonomous commands.
     * 
     * @param leftSpeed The speed for the left side (-1.0 to 1.0)
     * @param rightSpeed The speed for the right side (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftSpeed = limitSpeed(leftSpeed);
        rightSpeed = limitSpeed(rightSpeed);
        
        differentialDrive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Drive using arcade drive controls (forward/turn control).
     * Primary driving method during teleop.
     * 
     * @param forwardSpeed The forward/backward speed (-1.0 to 1.0)
     * @param rotationSpeed The rotation speed (-1.0 to 1.0)
     */
    public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
        forwardSpeed = limitSpeed(forwardSpeed);
        rotationSpeed = limitSpeed(rotationSpeed) * Constants.DriveConstants.TURN_SENSITIVITY;
        
        differentialDrive.arcadeDrive(forwardSpeed, rotationSpeed);
    }

    /**
     * Limits the speed to the safe range defined in constants.
     * Prevents motors from running too fast and applies deadband.
     * 
     * @param speed The requested speed
     * @return The limited speed
     */
    private double limitSpeed(double speed) {
        // Apply maximum speed limit
        double limitedSpeed = Math.min(Math.abs(speed), Constants.DriveConstants.MAX_DRIVE_SPEED) * Math.signum(speed);
        
        // Apply deadband to prevent small unintended movements
        if (Math.abs(limitedSpeed) < Constants.DriveConstants.DEADBAND) {
            return 0.0;
        }
        
        return limitedSpeed;
    }

    /**
     * Directly set motor voltages for precise control during autonomous.
     * Used by path following commands for accurate trajectory following.
     * 
     * @param leftVoltage The voltage for the left side motors
     * @param rightVoltage The voltage for the right side motors
     */
    public void setMotorVoltages(double leftVoltage, double rightVoltage) {
        leftLeadMotor.setVoltage(leftVoltage);
        rightLeadMotor.setVoltage(rightVoltage);
        
        // Feed the safety system to prevent timeouts
        differentialDrive.feed();
    }

    /**
     * Gets the current wheel speeds of the robot.
     * Used by trajectory following to maintain the correct path.
     * 
     * @return The current wheel speeds in meters per second
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()
        );
    }

    /**
     * Gets the left side distance traveled in meters.
     * 
     * @return Left distance in meters
     */
    public double getLeftDistanceMeters() {
        return leftEncoder.getPosition();
    }

    /**
     * Gets the right side distance traveled in meters.
     * 
     * @return Right distance in meters
     */
    public double getRightDistanceMeters() {
        return rightEncoder.getPosition();
    }

    /**
     * Returns the current robot heading from the gyro.
     * 
     * @return The heading in degrees (-180 to 180)
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Returns the gyro reading as a Rotation2d object.
     * Used by odometry for position tracking.
     * 
     * @return The current rotation
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Gets the current robot pose (position and rotation) on the field.
     * Used by autonomous commands to determine where the robot is.
     * 
     * @return The current pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     * Used at the start of autonomous to set initial position.
     * 
     * @param pose The pose to which to set the odometry
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(
            getRotation2d(),
            getLeftDistanceMeters(),
            getRightDistanceMeters(),
            pose
        );
    }

    /**
     * Resets the drive encoders to zero.
     * Used when starting a new movement sequence.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /**
     * Zeros the gyro heading.
     * Called at robot initialization and when re-orienting.
     */
    public void resetGyro() {
        gyro.reset();
        gyro.calibrate();
    }

    /**
     * Stops all drive motors.
     * Used when ending a command or in emergency situations.
     */
    public void stopMotors() {
        leftLeadMotor.stopMotor();
        rightLeadMotor.stopMotor();
    }

    /**
     * Returns the gyro's roll angle, which is useful for balancing.
     * 
     * @return The roll angle in degrees (-180 to 180)
     */
    public double getRoll() {
        return gyro.getRoll();
    }

    /**
     * Returns the gyro's pitch angle.
     * 
     * @return The pitch angle in degrees (-180 to 180)
     */
    public double getPitch() {
        return gyro.getPitch();
    }

    /**
     * Returns the rate of turn from the gyro.
     * Used to help smooth rotations.
     * 
     * @return The turn rate in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (Constants.DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Enables or disables brake mode on all drive motors.
     * Brake mode helps the robot stop quickly.
     * 
     * @param enable True for brake mode, false for coast mode
     */
    public void setBrakeMode(boolean enable) {
        CANSparkMax.IdleMode mode = enable ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast;
        leftLeadMotor.setIdleMode(mode);
        leftFollowMotor.setIdleMode(mode);
        rightLeadMotor.setIdleMode(mode);
        rightFollowMotor.setIdleMode(mode);
    }
    
    /**
     * Drives a specific distance forward at the specified speed.
     * Used by Drivetrain_GyroStraight command and autonomous routines.
     * 
     * @param distanceMeters The distance to drive in meters
     * @param speed The speed to drive at (-1.0 to 1.0)
     * @return True if the drive operation is complete
     */
    public boolean driveDistance(double distanceMeters, double speed) {
        double startLeftDistance = getLeftDistanceMeters();
        double startRightDistance = getRightDistanceMeters();
        
        double avgStartDistance = (startLeftDistance + startRightDistance) / 2.0;
        double avgCurrentDistance = (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
        
        double distanceTraveled = avgCurrentDistance - avgStartDistance;
        
        if (Math.abs(distanceTraveled) < Math.abs(distanceMeters)) {
            // Not at target distance yet, keep driving
            double direction = distanceMeters > 0 ? 1 : -1;
            arcadeDrive(direction * speed, 0);
            return false;
        } else {
            // Reached target distance, stop motors
            stopMotors();
            return true;
        }
    }
    
    /**
     * Turns the robot to a specific angle using the gyro for feedback.
     * Used by Drivetrain_GyroTurn command and autonomous routines.
     * 
     * @param targetAngleDegrees The target angle in degrees
     * @param turnSpeed The speed to turn at (0.0 to 1.0)
     * @return True if the turn operation is complete
     */
    public boolean turnToAngle(double targetAngleDegrees, double turnSpeed) {
        double currentAngle = getHeading();
        double angleError = targetAngleDegrees - currentAngle;
        
        // Normalize the error to -180 to 180 degrees
        angleError = Math.IEEEremainder(angleError, 360);
        
        // Determine turn direction based on the shortest path
        double turnDirection = Math.signum(angleError);
        
        // Check if we're within tolerance
        if (Math.abs(angleError) < Constants.DriveConstants.TURN_TOLERANCE_DEGREES) {
            stopMotors();
            return true;
        } else {
            // Scale turn speed for smoother approach
            double adjustedTurnSpeed = Math.max(
                turnSpeed * Math.min(1.0, Math.abs(angleError) / 60.0),
                Constants.DriveConstants.MIN_TURN_SPEED
            );
            
            // Turn in the appropriate direction
            arcadeDrive(0, turnDirection * adjustedTurnSpeed);
            return false;
        }
    }

    /**
     * Balance the robot on a charging station by driving until level.
     * Used during autonomous period for climb and balance.
     * 
     * @param balanceSpeed The speed to use when balancing (0.0 to 1.0)
     * @return True if the balance operation is complete
     */
    public boolean balanceRobot(double balanceSpeed) {
        double currentPitch = getPitch();
        
        // If we're nearly level, stop
        if (Math.abs(currentPitch) < Constants.DriveConstants.BALANCE_TOLERANCE_DEGREES) {
            stopMotors();
            setBrakeMode(true);  // Enable brake mode to hold position
            return true;
        }
        
        // Drive in the direction that will level the robot
        double direction = Math.signum(-currentPitch);
        double speed = balanceSpeed;
        
        // Reduce speed as we approach level for finer control
        if (Math.abs(currentPitch) < 10.0) {
            speed = balanceSpeed * 0.5;
        }
        
        arcadeDrive(direction * speed, 0);
        return false;
    }
}
