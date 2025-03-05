/*
 * ================================================================
 *   _____  ____  _   _  _____ _______       _   _ _______ _____  !
 *  / ____|/ __ \| \ | |/ ____|__   __|/\   | \ | |__   __/ ____| !
 * | |    | |  | |  \| | (___    | |  /  \  |  \| |  | | | (___   !
 * | |    | |  | | . ` |\___ \   | | / /\ \ | . ` |  | |  \___ \  !
 * | |____| |__| | |\  |____) |  | |/ ____ \| |\  |  | |  ____) | !
 *  \_____|\____/|_| \_|_____/   |_/_/    \_\_| \_|  |_| |_____/  !
 *                                                                 !
 * ================================================================
 */

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    //------------------------------------------
    // PHYSICAL ROBOT DIMENSIONS
    //------------------------------------------
    
    // WHEELS & CHASSIS
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // 6-inch wheels in meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double TRACK_WIDTH = Units.inchesToMeters(23); // Distance between left/right wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(32.3); // Distance between front/back wheels
    
    //------------------------------------------
    // MOTOR & SENSOR IDs
    //------------------------------------------
    
    // DRIVETRAIN MOTORS
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 2;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 3;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
    
    // BALL ARM SYSTEM MOTORS
    public static final int BALL_ARM_MOTOR_ID = 5; // NEO for arm movement
    public static final int BALL_GRIPPER_MOTOR_ID = 6; // NEO 550 for gripper wheels
    
    // HOOK SYSTEM MOTORS
    public static final int HOOK_MOTOR_ID = 7; // For linear actuator
    
    // DIGITAL INPUT PORTS
    public static final int BALL_ARM_UPPER_LIMIT_SWITCH_PORT = 0;
    public static final int BALL_ARM_LOWER_LIMIT_SWITCH_PORT = 1;
    public static final int BALL_DETECTOR_PING_PORT = 2; // Ultrasonic ping
    public static final int BALL_DETECTOR_ECHO_PORT = 3; // Ultrasonic echo
    public static final int HOOK_EXTENDED_LIMIT_SWITCH_PORT = 4; 
    public static final int HOOK_RETRACTED_LIMIT_SWITCH_PORT = 5;
    
    //------------------------------------------
    // USB PORTS FOR CONTROLLERS
    //------------------------------------------
    public static final int CONTROLLER_USB_PORT_ID = 0; // Driver controller
    public static final int OPERATOR_CONTROLLER_USB_PORT_ID = 1; // Operator controller
    
    //------------------------------------------
    // DRIVE SETTINGS - UPDATED FOR 16:1 RATIO!
    //------------------------------------------
    
    // HARDWARE CONFIG
    public static final double DRIVE_GEAR_RATIO = 16.0; // 16:1 ratio - OUR NEW UPGRADE!!
    public static final boolean REVERSE_LEFT_FRONT_MOTOR = true;
    public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
    public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_BACK_MOTOR = false;
    
    // SPEED SETTINGS - RETUNED FOR 16:1 RATIO
    public static final double DRIVE_NORMAL_SPEED = 0.85; // Increased from 0.8 for more power
    public static final double DRIVE_TURBO_SPEED = 1.0;   // MAXIMUM OVERDRIVE!!!
    public static final double DRIVE_PRECISION_SPEED = 0.35; // Reduced for finer control with higher torque
    public static final double MAX_SAFE_SPEED = 2.5; // Max speed in meters per second (adjusted for new ratio)
    
    //------------------------------------------
    // BALL ARM SETTINGS
    //------------------------------------------
    
    // HARDWARE CONFIG 
    public static final boolean BALL_ARM_MOTOR_INVERTED = false;
    public static final boolean BALL_GRIPPER_MOTOR_INVERTED = true;
    public static final double BALL_ARM_GEAR_RATIO = 16.0; // Updated to 16:1 with MAXPlanetary
    
    // ARM CONTROL CONSTANTS
    public static final double BALL_ARM_KP = 0.06; // Increased for 16:1 ratio
    public static final double BALL_ARM_MAX_SPEED = 0.65; // Increased for more arm power
    public static final double BALL_ARM_GRAVITY_FF = 0.12; // Higher feed forward for more torque
    
    // ARM POSITIONS (in encoder rotations)
    public static final double BALL_ARM_HOME_POSITION = 0.0; // Tucked in
    public static final double BALL_ARM_PICKUP_POSITION = -5.0; // Floor pickup
    public static final double BALL_ARM_SCORE_POSITION = 3.0; // Raised to score
    public static final double BALL_ARM_HORIZONTAL_POSITION = 1.5; // Horizontal level
    public static final double BALL_ARM_MIN_POSITION = -6.0; // Lower limit
    public static final double BALL_ARM_MAX_POSITION = 4.0; // Upper limit
    
    // GRIPPER SPEEDS
    public static final double BALL_GRIPPER_INTAKE_SPEED = 0.85;
    public static final double BALL_GRIPPER_HOLD_SPEED = 0.2;
    public static final double BALL_GRIPPER_RELEASE_SPEED = -0.7;
    
    // BALL DETECTION
    public static final double BALL_DETECTION_THRESHOLD_INCHES = 2.0;
    
    //------------------------------------------
    // HOOK SYSTEM SETTINGS
    //------------------------------------------
    
    // HARDWARE CONFIG
    public static final boolean HOOK_MOTOR_INVERTED = false;
    
    // HOOK CONTROL SPEEDS
    public static final double HOOK_EXTEND_MIN_SPEED = 0.2;
    public static final double HOOK_EXTEND_MAX_SPEED = 0.7;
    public static final double HOOK_RETRACT_SPEED = 0.5;
    
    // SAFETY LIMITS
    public static final double HOOK_MAX_CURRENT = 15.0; // Amps before emergency stop
    
    //------------------------------------------
    // CONTROLLER MAPPING
    //------------------------------------------
    
    // JOYSTICK AXES
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 5;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 4;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    
    // CONTROLLER BUTTONS - XBOX MAPPING
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
    
    //------------------------------------------
    // PID AND CONTROL CONSTANTS - RETUNED FOR 16:1!
    //------------------------------------------
    
    // DRIVETRAIN PID
    public static final double GYRO_TURN_KP = 0.009; // Increased for more torque
    public static final double TURNING_THRESHOLD_DEGREES = 2.5; // Tightened for better precision
    public static final double MAX_POWER_GYRO = 0.45; // Slightly increased
    
    // VELOCITY CONTROL - RETUNED FOR 16:1 RATIO
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

    //------------------------------------------
    // VISION SYSTEM SETTINGS
    //------------------------------------------

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
}
