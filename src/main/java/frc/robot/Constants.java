// Updated Constants.java with Ball Control System values
package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Constants class - WHERE THE MAGIC LIVES! 🧙‍♂️
 * 
 * This is like the robot's rulebook - all the numbers that make
 * our bot work perfectly! Change these with EXTREME CAUTION!
 * 
 * coded by paysean
 */
public final class Constants {
    // ========= DRIVE CONSTANTS =========
    // Physical Robot Constants //
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // 6-inch wheels in meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Measured in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(23); // Distance between centers of right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(32.3); // Distance between front and back wheels
    
    // Drivetrain Performance Settings
    public static final double DRIVE_NORMAL_SPEED = 0.8; // 80% of full power
    public static final double DRIVE_TURBO_SPEED = 1.0; // FULL POWER BABY!
    public static final double DRIVE_PRECISION_SPEED = 0.4; // Slow and precise
    public static final double MAX_SAFE_SPEED = 3.0; // Maximum safe speed in meters per second
    
    // ========= CONTROLLER SETTINGS =========
    public static final int CONTROLLER_USB_PORT_ID = 0; // Main driver controller USB port
    public static final int OPERATOR_CONTROLLER_USB_PORT_ID = 1; // Second controller for arm/gripper
    
    // Xbox Controller button/axis mapping
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 5;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 4;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    
    public static final int A_BUTTON = 1; // Bottom button
    public static final int B_BUTTON = 2; // Right button
    public static final int X_BUTTON = 3; // Left button
    public static final int Y_BUTTON = 4; // Top button
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BACK_BUTTON = 7;
    public static final int START_BUTTON = 8;
    public static final int LEFT_STICK_BUTTON = 9;
    public static final int RIGHT_STICK_BUTTON = 10;
    
    // ========= BALL ARM SYSTEM =========
    // Motor IDs
    public static final int BALL_ARM_MOTOR_ID = 5; // NEO motor for arm
    public static final int BALL_GRIPPER_MOTOR_ID = 6; // NEO 550 for gripper wheels
    
    // Sensor ports
    public static final int BALL_ARM_UPPER_LIMIT_SWITCH_PORT = 0; // DIO port for upper limit
    public static final int BALL_ARM_LOWER_LIMIT_SWITCH_PORT = 1; // DIO port for lower limit
    public static final int BALL_DETECTOR_PING_PORT = 2; // Ultrasonic ping
    public static final int BALL_DETECTOR_ECHO_PORT = 3; // Ultrasonic echo
    
    // Arm control constants
    public static final boolean BALL_ARM_MOTOR_INVERTED = false;
    public static final boolean BALL_GRIPPER_MOTOR_INVERTED = false;
    
    public static final double BALL_ARM_KP = 0.05; // P gain for position control
    public static final double BALL_ARM_MAX_SPEED = 0.7; // Maximum arm speed
    public static final double BALL_ARM_GRAVITY_FF = 0.1; // Feed forward to fight gravity
    
    // Arm positions (in encoder rotations)
    public static final double BALL_ARM_HOME_POSITION = 0.0;
    public static final double BALL_ARM_PICKUP_POSITION = -5.0;
    public static final double BALL_ARM_SCORE_POSITION = 3.0;
    public static final double BALL_ARM_HORIZONTAL_POSITION = 1.5;
    
    // Gripper speeds
    public static final double BALL_GRIPPER_INTAKE_SPEED = 0.85;
    public static final double BALL_GRIPPER_HOLD_SPEED = 0.2;
    public static final double BALL_GRIPPER_RELEASE_SPEED = -0.7;
    
    // Ball detection
    public static final double BALL_DETECTION_THRESHOLD_INCHES = 2.0;
    
    // ========= MOTOR IDs =========
    // CAN IDs for SparkMax controllers
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 2;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 3;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
    
    // ========= DRIVETRAIN CONFIG =========
    public static final double DRIVE_GEAR_RATIO = 10.71; // Toughbox Mini 10.71:1 ratio
    public static final boolean REVERSE_LEFT_FRONT_MOTOR = true;
    public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
    public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_BACK_MOTOR = false;
    
    // PID and motion control
    public static final double GYRO_TURN_KP = 0.007;
    public static final double TRACKED_TAG_ROATION_KP = 0.3;
    public static final double TRACKED_TAG_FORWARD_DRIVE_KP = 0.4;
    public static final double TRACKED_TAG_STRAFE_DRIVE_KP = 0.5;
    public static final double TURNING_THRESHOLD_DEGREES = 3.0;
    public static final double MAX_POWER_GYRO = 0.4;
    
    public static final double kP_FRONT_RIGHT_VELOCITY = 0.0010269;
    public static final double kP_FRONT_LEFT_VELOCITY = 0.0010269;
    public static final double kP_BACK_RIGHT_VELOCITY = 0.0010269;
    public static final double kP_BACK_LEFT_VELOCITY = 0.0010269;
    public static final double kP_X_CONTROLLER = 9.6421;
    public static final double kP_Y_CONTROLLER = 9.6421;
    public static final double kP_THETA_CONTROLLER = 9.6421;
    
    public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI;
    public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2*Math.PI;
}
