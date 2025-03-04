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
 * 
 * WHERE ALL THE MAGIC NUMBERS LIVE!!!
 * 
 * If you change these, YOU BREAK EVERYTHING!
 * IF YOU ADD NEW ONES, COMMENT THEM!!!
 * 
 * All units in: METERS, SECONDS, RADIANS (unless specified)
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
    // DRIVE SETTINGS
    //------------------------------------------
    
    // HARDWARE CONFIG
    public static final double DRIVE_GEAR_RATIO = 10.71; // 10.71:1 ratio from Toughbox Mini
    public static final boolean REVERSE_LEFT_FRONT_MOTOR = true;
    public static final boolean REVERSE_LEFT_BACK_MOTOR = true;
    public static final boolean REVERSE_RIGHT_FRONT_MOTOR = false;
    public static final boolean REVERSE_RIGHT_BACK_MOTOR = false;
    
    // SPEED SETTINGS
    public static final double DRIVE_NORMAL_SPEED = 0.8; // 80% of full power
    public static final double DRIVE_TURBO_SPEED = 1.0; // MAXIMUM OVERDRIVE!!!
    public static final double DRIVE_PRECISION_SPEED = 0.4; // Slow and precise
    public static final double MAX_SAFE_SPEED = 3.0; // Max speed in meters per second
    
    //------------------------------------------
    // BALL ARM SETTINGS
    //------------------------------------------
    
    // HARDWARE CONFIG 
    public static final boolean BALL_ARM_MOTOR_INVERTED = false; // Set based on motor mounting
    public static final boolean BALL_GRIPPER_MOTOR_INVERTED = true; // Set to grip properly
    public static final double BALL_ARM_GEAR_RATIO = 25.0; // 25:1 from MAXPlanetary
    
    // ARM CONTROL CONSTANTS
    public static final double BALL_ARM_KP = 0.05; // P gain for position control
    public static final double BALL_ARM_MAX_SPEED = 0.6; // Max arm speed (0-1)
    public static final double BALL_ARM_GRAVITY_FF = 0.1; // Feed forward to fight gravity
    
    // ARM POSITIONS (in encoder rotations)
    public static final double BALL_ARM_HOME_POSITION = 0.0; // Tucked in
    public static final double BALL_ARM_PICKUP_POSITION = -5.0; // Floor pickup
    public static final double BALL_ARM_SCORE_POSITION = 3.0; // Raised to score
    public static final double BALL_ARM_HORIZONTAL_POSITION = 1.5; // Horizontal level
    public static final double BALL_ARM_MIN_POSITION = -6.0; // Lower limit
    public static final double BALL_ARM_MAX_POSITION = 4.0; // Upper limit
    
    // GRIPPER SPEEDS
    public static final double BALL_GRIPPER_INTAKE_SPEED = 0.85; // Ball intake
    public static final double BALL_GRIPPER_HOLD_SPEED = 0.2; // Hold ball without crushing
    public static final double BALL_GRIPPER_RELEASE_SPEED = -0.7; // Launch ball
    
    // BALL DETECTION
    public static final double BALL_DETECTION_THRESHOLD_INCHES = 2.0; // Distance to detect ball
    
    //------------------------------------------
    // HOOK SYSTEM SETTINGS
    //------------------------------------------
    
    // HARDWARE CONFIG
    public static final boolean HOOK_MOTOR_INVERTED = false;
    
    // HOOK CONTROL SPEEDS
    public static final double HOOK_EXTEND_MIN_SPEED = 0.2; // Starting speed
    public static final double HOOK_EXTEND_MAX_SPEED = 0.7; // Full extension speed
    public static final double HOOK_RETRACT_SPEED = 0.5; // Retraction speed
    
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
    // PID AND CONTROL CONSTANTS
    //------------------------------------------
    
    // DRIVETRAIN PID
    public static final double GYRO_TURN_KP = 0.007;
    public static final double TURNING_THRESHOLD_DEGREES = 3.0;
    public static final double MAX_POWER_GYRO = 0.4;
    
    // VELOCITY CONTROL
    public static final double kP_FRONT_RIGHT_VELOCITY = 0.0010269;
    public static final double kP_FRONT_LEFT_VELOCITY = 0.0010269;
    public static final double kP_BACK_RIGHT_VELOCITY = 0.0010269;
    public static final double kP_BACK_LEFT_VELOCITY = 0.0010269;
    
    // PATH FOLLOWING
    public static final double kP_X_CONTROLLER = 9.6421;
    public static final double kP_Y_CONTROLLER = 9.6421;
    public static final double kP_THETA_CONTROLLER = 9.6421;
    
    // MOTION CONSTRAINTS
    public static final double kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2*Math.PI;
    public static final double kMAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2*Math.PI;

   //------------------------------------------
   // VISION SYSTEM SETTINGS
   //------------------------------------------

   // TRACKING CONSTANTS
    public static final double TRACKED_TAG_ROATION_KP = 0.015; // Rotation control gain
    public static final double TRACKED_TAG_FORWARD_DRIVE_KP = 0.3; // Forward control gain
    public static final double TRACKED_TAG_STRAFE_DRIVE_KP = 0.4; // Strafe control gain

   // APRILTAG CONFIG
    public static final double APRILTAG_ROTATION_POWER_CAP = 0.4; // Max rotation power
    public static final double APRILTAG_FORWARD_POWER_CAP = 0.4; // Max forward power
    public static final double APRILTAG_STRAFE_POWER_CAP = 0.4; // Max strafe power
    public static final double APRILTAG_TRACKING_DISTANCE_THRESHOLD = 0.3; // Meters

   // CAMERA SETTINGS
    public static final int CAMERA_WIDTH = 640; // Camera resolution width
    public static final int CAMERA_HEIGHT = 480; // Camera resolution height
    public static final double CAMERA_FOV = 68.5; // Field of view in degrees

    //------------------------------------------
    // BALL DETECTION & TRACKING
    //------------------------------------------

   // Ball detection thresholds
   public static final double BALL_MIN_AREA = 0.01;        // Minimum area of ball in image (0-1)
   public static final double BALL_DETECTION_DISTANCE = 5.0; // Maximum detection distance in meters

   // PID values for tracking
   public static final double BALL_TRACKING_KP = 0.015; // P gain for tracking
   public static final double BALL_APPROACH_KP = 0.3;   // P gain for approach
}
