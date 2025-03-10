package frc.robot;

/**
 * Constants.java - Central configuration for Team 7221's Reefscape robot
 * 
 * This file contains all the numerical constants that configure how the robot operates.
 * Constants are organized into logical sections by subsystem or function.
 * 
 * ┌───────────────────────────────────────────────────────────┐
 * │  TEAM 7221 - ROBOT CONFIGURATION CONSTANTS                │
 * │  Organized by subsystem for easy reference                │
 * └───────────────────────────────────────────────────────────┘
 */
public final class Constants {

    /**
     * Global robot performance settings
     */
    public static final class Performance {
        // Battery management thresholds
        public static final double BATTERY_BROWNOUT_THRESHOLD = 7.0;  // Critical voltage
        public static final double BATTERY_WARNING_THRESHOLD = 10.5;  // Low voltage warning
        
        // Loop timing constraints
        public static final double LOOP_TIME_WARNING = 0.018;  // Time in seconds (18ms) before warning
        
        // Status indicators
        public static final int STATUS_OK = 0;
        public static final int STATUS_WARNING = 1;
        public static final int STATUS_CRITICAL = 2;
    }

    /**
     * Physical dimensions of the robot
     */
    public static final class Dimensions {
        // Drivetrain geometry
        public static final double TRACK_WIDTH = 0.57;          // Distance between left/right wheels (meters)
        public static final double WHEEL_DIAMETER = 0.1524;     // 6 inches in meters
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        
        // Robot frame dimensions
        public static final double ROBOT_LENGTH = 0.8128;       // 32 inches in meters
        public static final double ROBOT_WIDTH = 0.6858;        // 27 inches in meters
    }

    /**
     * Controller and input settings
     */
    public static final class Controls {
        // Controller USB ports
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        
        // Input processing
        public static final double JOYSTICK_DEADBAND = 0.08;   // Eliminates drift in joystick center
        
        /**
         * Apply input curve to improve drivability
         * Makes controls more precise at low speeds while maintaining max power
         */
        public static double applyInputCurve(double input) {
            // Square input while preserving sign for intuitive feel
            return Math.signum(input) * (input * input);
        }
    }

    /**
     * Electrical system port assignments
     */
    public static final class Electrical {
        // CAN bus IDs for drivetrain motors (1-4)
        public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 1;
        public static final int LEFT_REAR_DRIVE_MOTOR_ID = 2;
        public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 3;
        public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 4;
        
        // CAN bus IDs for mechanism motors (5-7)
        public static final int BALL_ARM_EXTENSION_MOTOR_ID = 5;
        public static final int BALL_ARM_GRIPPER_MOTOR_ID = 6;
        public static final int HOOK_MOTOR_ID = 7;
        
        // Digital input ports for limit switches
        public static final int BALL_ARM_EXTENDED_LIMIT_PORT = 0;
        public static final int BALL_ARM_RETRACTED_LIMIT_PORT = 1;
        public static final int HOOK_EXTENDED_LIMIT_SWITCH_PORT = 2;
        public static final int HOOK_RETRACTED_LIMIT_SWITCH_PORT = 3;
        
        // Digital I/O ports for sensors
        public static final int BALL_DETECTOR_PING_PORT = 4;  // Ultrasonic sensor output
        public static final int BALL_DETECTOR_ECHO_PORT = 5;  // Ultrasonic sensor input
        
        // Current limits for motors
        public static final int MAX_CURRENT_DRIVE_MOTOR = 40;    // Amps
        public static final int MAX_CURRENT_MECHANISM_MOTOR = 30; // Amps
    }

    /**
     * Drivetrain configuration
     */
    public static final class Drivetrain {
        // // Encoder configuration - Uncomment if you add encoders
        // public static final boolean LEFT_ENCODER_INVERTED = false;
        // public static final boolean RIGHT_ENCODER_INVERTED = true;
        // public static final double DISTANCE_PER_PULSE = Dimensions.WHEEL_CIRCUMFERENCE / 2048.0;
        
        // Speed modes - percentage of max power
        public static final double DRIVE_PRECISION_SPEED = 0.35;  // 35% - for fine control
        public static final double DRIVE_NORMAL_SPEED = 0.85;     // 85% - for regular driving
        public static final double DRIVE_TURBO_SPEED = 1.0;       // 100% - for maximum speed
        
        // Control parameters
        public static final double THROTTLE_SLEW_RATE = 2.0;  // How quickly to ramp up from 0 to 1 (seconds)
        public static final double TURN_SLEW_RATE = 2.0;      // How quickly turning can change
        
        // // Uncomment if using PID control
        // public static final double kP_DRIVE_VEL = 0.1;        // Proportional gain for velocity control
        
        // Motor controller configuration
        public static final double VOLTAGE_COMPENSATION = 11.0;   // Target voltage for compensation
        public static final double OPEN_LOOP_RAMP_RATE = 0.2;     // Seconds from 0 to full power
    }

    /**
     * Ball Arm system configuration
     */
    public static final class BallArm {
        // Motor configuration
        public static final int EXTENSION_MOTOR_ID = Electrical.BALL_ARM_EXTENSION_MOTOR_ID;
        public static final int GRIPPER_MOTOR_ID = Electrical.BALL_ARM_GRIPPER_MOTOR_ID;
        public static final boolean EXTENSION_MOTOR_INVERTED = false;
        public static final boolean GRIPPER_MOTOR_INVERTED = true;
        
        // Position limits in encoder rotations
        public static final double MIN_POSITION = 0.0;        // Fully retracted
        public static final double MAX_POSITION = 18.0;       // Fully extended
        public static final double HOME_POSITION = 0.5;       // Stowed position
        public static final double PICKUP_POSITION = 14.0;    // Position for floor pickup
        public static final double SCORE_POSITION = 16.0;     // Position for scoring
        
        // Speed settings - controls how fast the arm moves
        public static final double MAX_SPEED = 0.7;             // Maximum arm movement speed
        public static final double GRIPPER_INTAKE_SPEED = 0.8;  // Speed for ball intake
        public static final double GRIPPER_HOLD_SPEED = 0.2;    // Speed to maintain grip
        public static final double GRIPPER_RELEASE_SPEED = 0.8; // Speed for ball release
        
        // // PID Control - Uncomment if using PID for arm positioning
        // public static final double POSITION_KP = 0.1;     // Proportional gain
        // public static final double POSITION_KI = 0.0;     // Integral gain
        // public static final double POSITION_KD = 0.01;    // Derivative gain
        
        // Ball detection with ultrasonic sensor
        public static final double DETECTION_THRESHOLD_INCHES = 2.0;  // Distance indicating ball present
        
        // Safety thresholds
        public static final double STALL_CURRENT_THRESHOLD = 25.0;    // Current indicating jam (amps)
    }

    /**
     * Hook system configuration
     */
    public static final class Hook {
        // Motor configuration
        public static final int MOTOR_ID = Electrical.HOOK_MOTOR_ID;
        public static final boolean MOTOR_INVERTED = false;
        
        // Speed settings
        public static final double EXTEND_MIN_SPEED = 0.3;    // Initial extension speed
        public static final double EXTEND_MAX_SPEED = 0.7;    // Full extension speed
        public static final double RETRACT_SPEED = 0.8;       // Retraction speed
        
        // Safety thresholds
        public static final double MAX_CURRENT = 30.0;        // Current limit (amps)
    }

    /**
     * Vision system configuration
     */
    public static final class Vision {
        // Camera mounting position
        public static final double CAMERA_HEIGHT_METERS = 0.45;      // Camera height from floor
        public static final double CAMERA_PITCH_RADIANS = 0.15;      // Downward tilt (~8.6°)
        
        // Target heights
        public static final double BALL_HEIGHT_METERS = 0.12;        // Ball height from floor
        
        // Pipeline indexes
        public static final int BALL_PIPELINE_INDEX = 0;             // Pipeline for ball detection
        public static final int APRILTAG_PIPELINE_INDEX = 1;         // Pipeline for AprilTag detection
    }

    /**
     * Autonomous routine configuration
     */
    public static final class Autonomous {
        // Basic autonomous parameters
        public static final double DEFAULT_AUTO_SPEED = 0.5;        // Default speed for autonomous
        public static final double DEFAULT_AUTO_DRIVE_TIME = 2.0;   // Default drive time in seconds
        
        // // Advanced autonomous parameters - Uncomment if needed
        // public static final double GYRO_TURN_KP = 0.01;     // Proportional gain for turning
        // public static final double DRIVE_KP = 0.1;          // Proportional gain for driving
        // public static final double TURNING_THRESHOLD_DEGREES = 2.0;  // Degrees of error allowed
        
        // Constants for specific autonomous routines
        public static final double MAX_POWER_GYRO = 0.6;      // Maximum power for Gyro commands
    }
}
