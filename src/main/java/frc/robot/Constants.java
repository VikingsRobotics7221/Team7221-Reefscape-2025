/*
 * ═════════════════════════════════════════════════════════════
 *  ____  ____  _   _  ____ ____    _   _ ____ ____  
 * / ___|| __ )| \ | |/ ___|_   _|/\   | \ | |_   _|/ ____|
 * | |    |  _ \|  \| | (___   | | /  \  |  \| | | | | (___  
 * | |    | |_) | |\  |\___ \  | |/ /\ \ | . ` | | |  \___ \ 
 * | |___ |  _ <| |\  |____) | | / ____ \| |\  | | |  ____) |
 *  \____||_| \_\_| \_|_____/  |_/_/    \_\_| \_| |_| |_____/ 
 * ═════════════════════════════════════════════════════════════
 * 
 * TEAM 7221 - REEFSCAPE 2025 - MASTER CONSTANTS FILE
 * 
 * These constants control EVERYTHING on our robot. They are the
 * difference between victory and defeat. I spent hours tuning 
 * these values to absolute perfection!
 * 
 * Last Updated: March 2025
 * Coded by paysean
 */

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a centralized location for all robot configuration
 * values. These values define how the robot hardware is configured and how the
 * control systems behave.
 * 
 * ALWAYS TEST AFTER CHANGING THESE VALUES! Even small adjustments can
 * completely change robot behavior.
 */
public final class Constants {
    //----
    // PHYSICAL ROBOT DIMENSIONS
    //----
    
    // DRIVETRAIN DIMENSIONS
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // 6-inch wheels in meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double TRACK_WIDTH = Units.inchesToMeters(23); // Distance between left/right wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(32.3); // Distance between front/back wheels
    
    // ROBOT DIMENSIONS 
    public static final double ROBOT_LENGTH = Units.inchesToMeters(32); // Frame perimeter length
    public static final double ROBOT_WIDTH = Units.inchesToMeters(27); // Frame perimeter width
    public static final double ROBOT_HEIGHT = Units.inchesToMeters(24); // Base height without mechanisms
    public static final double ARM_MAX_REACH = Units.inchesToMeters(18); // Maximum extension of arm
    
    //----
    // MOTOR & SENSOR IDs
    //----
    
    // DRIVETRAIN MOTORS (SPARK MAX CAN IDs)
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 2;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 3;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
    
    // BALL ARM SYSTEM MOTORS
    public static final int BALL_ARM_EXTENSION_MOTOR_ID = 5; // NEO motor for drawer slide
    public static final int BALL_GRIPPER_MOTOR_ID = 6; // NEO 550 for gripper wheels
    
    // HOOK SYSTEM MOTORS
    public static final int HOOK_MOTOR_ID = 7; // Motor for actuator
    
    // DIGITAL INPUT PORTS (DIO on RoboRIO)
    public static final int BALL_ARM_EXTENDED_LIMIT_PORT = 0; // Limit switch for full extension
    public static final int BALL_ARM_RETRACTED_LIMIT_PORT = 1; // Limit switch for full retraction
    public static final int BALL_DETECTOR_PING_PORT = 2; // Ultrasonic ping
    public static final int BALL_DETECTOR_ECHO_PORT = 3; // Ultrasonic echo
    public static final int HOOK_EXTENDED_LIMIT_SWITCH_PORT = 4; 
    public static final int HOOK_RETRACTED_LIMIT_SWITCH_PORT = 5;
    
    // ANALOG INPUT PORTS
    public static final int PRESSURE_SENSOR_PORT = 0; // Pressure sensor for pneumatics
    public static final int BATTERY_MONITOR_PORT = 1; // External battery voltage monitor
    
    //----
    // USB PORTS FOR CONTROLLERS
    //----
    public static final int CONTROLLER_USB_PORT_ID = 0; // Driver controller
    public static final int OPERATOR_CONTROLLER_USB_PORT_ID = 1; // Operator controller
    
    // CONTROLLER AXIS IDs
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1; // Left Y axis
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 4; // Right X axis
    public static final double JOYSTICK_DEADBAND = 0.1; // Ignore inputs below this value
    
    //----
    // DRIVE SETTINGS - TUNED FOR 16:1 RATIO!
    //----
    
    // HARDWARE CONFIG
    public static final double DRIVE_GEAR_RATIO = 16.0; // 16:1 ratio for MAXIMUM TORQUE!
    public static final boolean REVERSE_LEFT_FRONT_MOTOR = true;
    public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
    public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_BACK_MOTOR = false;
    
    // SPEED SETTINGS - CAREFULLY TUNED!
    public static final double DRIVE_NORMAL_SPEED = 0.85; // 85% power in normal mode
    public static final double DRIVE_TURBO_SPEED = 1.0; // 100% POWER - USE WITH CAUTION!
    public static final double DRIVE_PRECISION_SPEED = 0.5; // 50% power for fine control
    public static final double DRIVE_AUTONOMOUS_SPEED = 0.7; // 70% power for autonomous
    
    // RAMP RATES - PREVENTS BROWNOUTS & WHEELIES
    public static final double OPEN_LOOP_RAMP_RATE = 0.2; // 200ms ramp to full power (teleop)
    public static final double CLOSED_LOOP_RAMP_RATE = 0.1; // 100ms ramp to full power (autonomous)
    
    // VOLTAGE COMPENSATION - Keeps behavior consistent as battery drains
    public static final double VOLTAGE_COMPENSATION = 11.0; // Set to 11V for consistent performance
    
    //----
    // BALL ARM SYSTEM SETTINGS
    //----
    
    // ARM CONFIGURATION
    public static final boolean BALL_ARM_EXTENSION_MOTOR_INVERTED = false; // Direction of extension
    public static final boolean BALL_GRIPPER_MOTOR_INVERTED = false; // Direction of gripper
    
    // ARM POSITION LIMITS
    public static final double BALL_ARM_MIN_POSITION = 0.0; // Fully retracted
    public static final double BALL_ARM_MAX_POSITION = 6.0; // Fully extended
    public static final double BALL_ARM_HOME_POSITION = 0.1; // Safe home position
    public static final double BALL_ARM_PICKUP_POSITION = 3.0; // Position for floor pickup
    public static final double BALL_ARM_SCORE_POSITION = 5.5; // Position for scoring
    
    // ARM SPEED & POWER SETTINGS
    public static final double BALL_ARM_MAX_SPEED = 0.7; // Maximum arm extension speed
    public static final double BALL_ARM_STALL_CURRENT_THRESHOLD = 25.0; // Amps, warning threshold
    
    // ARM PID CONTROL - TUNED FOR 16:1 GEARBOX
    public static final double BALL_ARM_KP = 0.1; // Proportional gain
    public static final double BALL_ARM_KI = 0.0; // Integral gain
    public static final double BALL_ARM_KD = 0.0; // Derivative gain
    public static final double BALL_ARM_KF = 0.0; // Feed-forward gain
    
    // GRIPPER SETTINGS
    public static final double BALL_GRIPPER_INTAKE_SPEED = 0.8; // Speed for intaking balls
    public static final double BALL_GRIPPER_HOLD_SPEED = 0.2; // Speed to hold balls securely
    public static final double BALL_GRIPPER_RELEASE_SPEED = -1.0; // Speed to eject balls
    
    // DETECTION SETTINGS
    public static final double BALL_DETECTION_THRESHOLD_INCHES = 2.0; // Ultrasonic detection distance
    
    //----
    // HOOK SYSTEM SETTINGS
    //----
    
    // HOOK MOTOR SETTINGS
    public static final boolean HOOK_MOTOR_INVERTED = false; // Direction of extension
    public static final double HOOK_EXTEND_SPEED = 0.7; // Speed for extending hook
    public static final double HOOK_RETRACT_SPEED = -0.7; // Speed for retracting hook
    
    // HOOK POSITION LIMITS
    public static final double HOOK_EXTENSION_TIMEOUT = 3.0; // Maximum seconds for extension
    public static final double HOOK_RETRACTION_TIMEOUT = 3.0; // Maximum seconds for retraction
    
    //----
    // AUTONOMOUS SETTINGS
    //----
    
    // AUTONOMOUS TIMING
    public static final double AUTO_DEFAULT_TIMEOUT = 5.0; // Default timeout for commands
    public static final double AUTO_SHORT_TIMEOUT = 2.0; // Short timeout for quick movements
    public static final double AUTO_LONG_TIMEOUT = 10.0; // Long timeout for complex actions
    
    // PID AND CONTROL CONSTANTS
    public static final double GYRO_TURN_KP = 0.009; // Proportional gain for gyro turns
    public static final double TURNING_THRESHOLD_DEGREES = 2.5; // Acceptable angle error
    public static final double MAX_POWER_GYRO = 0.45; // Max power during gyro turns
    
    // VELOCITY CONTROL - TUNED FOR 16:1 RATIO
    public static final double kP_FRONT_RIGHT_VELOCITY = 0.00125;
    public static final double kP_FRONT_LEFT_VELOCITY = 0.00125;
    public static final double kP_BACK_RIGHT_VELOCITY = 0.00125;
    public static final double kP_BACK_LEFT_VELOCITY = 0.00125;
    
    // PATH FOLLOWING
    public static final double kP_X_CONTROLLER = 10.0;
    public static final double kP_Y_CONTROLLER = 10.0;
    public static final double kP_THETA_CONTROLLER = 10.0;
    
    // MOTION CONSTRAINTS
    public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 1.8*Math.PI; // Reduced for 16:1
    public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 1.8*Math.PI;
    
    //----
    // VISION SYSTEM SETTINGS
    //----
    
    // TRACKING CONSTANTS
    public static final double TRACKED_TAG_ROATION_KP = 0.015;
    public static final double TRACKED_TAG_FORWARD_DRIVE_KP = 0.3;
    public static final double TRACKED_TAG_STRAFE_DRIVE_KP = 0.4;
    
    // APRILTAG CONFIG
    public static final double APRILTAG_ROTATION_POWER_CAP = 0.4;
    public static final double APRILTAG_FORWARD_POWER_CAP = 0.4;
    public static final double APRILTAG_STRAFE_POWER_CAP = 0.4;
    public static final double APRILTAG_TRACKING_DISTANCE_THRESHOLD = 0.3;
    
    // CAMERA SETTINGS
    public static final int CAMERA_WIDTH = 640;
    public static final int CAMERA_HEIGHT = 480;
    public static final double CAMERA_FOV = 68.5;
    public static final int CAMERA_FPS = 30;
    
    //----
    // PERFORMANCE MONITORING
    //----
    
    // BATTERY MANAGEMENT
    public static final double BATTERY_BROWNOUT_THRESHOLD = 7.0; // Volts
    public static final double BATTERY_WARNING_THRESHOLD = 11.0; // Volts
    public static final double BATTERY_GOOD_THRESHOLD = 12.0; // Volts
    
    // THERMAL PROTECTION
    public static final double MOTOR_TEMPERATURE_WARNING = 70.0; // Degrees C
    public static final double MOTOR_TEMPERATURE_CRITICAL = 85.0; // Degrees C
    
    // LOOP TIMING
    public static final double TARGET_LOOP_TIME = 0.02; // 20ms (50Hz)
    public static final double LOOP_TIME_WARNING = 0.025; // 25ms (warn if loop takes too long)
    
    //----
    // COMPETITION STRATEGY SETTINGS
    //----
    
    // GAME PIECE COLLECTION
    public static final int BALL_COLLECTION_PRIORITY = 1; // High priority
    public static final double COLLECTION_DISTANCE_THRESHOLD = 3.0; // Meters
    public static final double COLLECTION_AUTO_TRIGGER_DISTANCE = 1.0; // Auto-collect within this distance
    
    // SCORING STRATEGY
    public static final double ENDGAME_TIME_THRESHOLD = 30.0; // Seconds remaining to focus on endgame
    public static final int HOOK_ENGAGE_PRIORITY = 2; // Medium-high priority in endgame
    
    // DEFENSE SETTINGS
    public static final double DEFENSE_SPEED_MULTIPLIER = 0.8; // 80% speed when playing defense
    public static final double DEFENSIVE_POSITION_TOLERANCE = 0.3; // Meters - positioning accuracy
    
    //----
    // SYSTEM STATUS CODES
    //----
    
    // Status codes for system health monitoring
    public static final int STATUS_OK = 0;
    public static final int STATUS_WARNING = 1;
    public static final int STATUS_ERROR = 2;
    public static final int STATUS_CRITICAL = 3;
    
    // Special code to indicate we're in competition mode
    // This affects certain behaviors (less console output, more aggressive performance)
    public static final boolean COMPETITION_MODE = false; // Set to true at competition!
}
