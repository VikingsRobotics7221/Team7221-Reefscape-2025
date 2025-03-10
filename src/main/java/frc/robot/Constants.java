package frc.robot;

/**
 * Constants.java - The one place to adjust all your robot's numbers
 * 
 * HOW TO USE THIS FILE:
 * 1. Find the section for the part of your robot you want to adjust
 * 2. Change the numbers in the SETUP ZONE for that section
 * 3. Save the file
 * 4. Rebuild and deploy your code
 * 
 * IMPORTANT: Numbers with "final" in front can't be changed while the robot is running!
 */
public final class Constants {

    /**
     * Global robot settings that apply to everything
     */
    public static final class Robot {
        // How fast the code repeats (don't change unless you know what you're doing)
        public static final double LOOP_TIME_SECONDS = 0.02; // 50 times per second
        
        // Main power settings
        public static final double LOW_BATTERY_VOLTAGE = 11.0; // Warn when battery gets this low
    }

    //=================================================================
    //   DRAWER SLIDE ARM SYSTEM - SETUP ZONE
    //=================================================================
    /**
     * All settings for the Drawer Slide Arm System
     * CHANGE THESE VALUES to match your specific hardware
     */
    public static final class DrawerSlideArm {
        // STEP 1: GIVE YOUR SYSTEM A NAME
        public static final String SYSTEM_NAME = "Ball Grabber 3000"; // Name it whatever you want
        
        // STEP 2: MOTOR SETUP - Which motors and what ports?
        
        // Motor that extends/retracts the arm
        // PICK ONE: "SPARK_MAX", "TALON_SRX", "VICTOR_SPX", "PWM"
        public static final String EXTENSION_MOTOR_TYPE = "SPARK_MAX";
        public static final int EXTENSION_MOTOR_PORT = 12; // Port number
        public static final boolean EXTENSION_MOTOR_REVERSED = false; // Is it backward?
        
        // Motor that grabs/releases the game piece
        // PICK ONE: "SPARK_MAX", "TALON_SRX", "VICTOR_SPX", "PWM"
        public static final String GRIPPER_MOTOR_TYPE = "SPARK_MAX";
        public static final int GRIPPER_MOTOR_PORT = 13; // Port number
        public static final boolean GRIPPER_MOTOR_REVERSED = true; // Is it backward?
        
        // STEP 3: LIMIT SWITCH SETUP - Stops arm from going too far
        // Are you using limit switches? (true/false)
        public static final boolean USING_LIMIT_SWITCHES = true;
        public static final int EXTENDED_LIMIT_SWITCH_PORT = 0; // When fully out
        public static final int RETRACTED_LIMIT_SWITCH_PORT = 1; // When fully in
        
        // STEP 4: GAME PIECE DETECTION - How do you know you have a ball/cube?
        // PICK ONE: "ULTRASONIC", "BEAM_BREAK", "LIMIT_SWITCH", "NONE"
        public static final String GAME_PIECE_DETECTOR_TYPE = "ULTRASONIC";
        public static final int DETECTOR_PORT_A = 2; // First port (ping for ultrasonic)
        public static final int DETECTOR_PORT_B = 3; // Second port (echo for ultrasonic)
        public static final double DETECTION_THRESHOLD = 2.0; // Distance in inches that means "piece present"
        
        // STEP 5: ARM POSITIONS - How far should the arm move to each position?
        public static final double HOME_POSITION = 0.5; // Nearly all the way in
        public static final double PICKUP_POSITION = 14.0; // Extended to grab pieces
        public static final double SCORE_POSITION = 16.0; // Extended to score
        
        // STEP 6: SPEED SETTINGS - How fast should things move?
        public static final double MAX_EXTENSION_SPEED = 0.7; // Max speed (0-1)
        public static final double INTAKE_SPEED = 0.8; // Speed to grab game pieces (0-1)
        public static final double HOLD_SPEED = 0.2; // Speed to hold game pieces (0-1)
        public static final double RELEASE_SPEED = -0.8; // Speed to release game pieces (-1 to 0)
        
        // STEP 7: SAFETY SETTINGS - Prevent damage to your robot
        public static final double MAX_MOTOR_CURRENT = 30.0; // Max amps before slowing down
        public static final double STALL_DETECTION_THRESHOLD = 25.0; // Amps that indicate a jam
        
        // STEP 8: FINE-TUNING (ask your mentor if you're not sure)
        // Controls how precisely the arm moves to positions
        public static final double POSITION_TOLERANCE = 0.25; // How close is "close enough" (inches)
        public static final double POSITION_KP = 0.1; // How hard to try reaching target
        public static final double POSITION_KI = 0.0; // Fixes small errors
        public static final double POSITION_KD = 1.0; // Prevents overshooting
    }
    //=================================================================
    //   END DRAWER SLIDE ARM SYSTEM - SETUP ZONE
    //=================================================================

    /**
     * Drivetrain settings
     * Controls how your robot drives around the field
     */
    public static final class Drivetrain {
        // Motor ports
        public static final int LEFT_LEADER_PORT = 1;
        public static final int LEFT_FOLLOWER_PORT = 2;
        public static final int RIGHT_LEADER_PORT = 3;
        public static final int RIGHT_FOLLOWER_PORT = 4;
        
        // Are the motors flipped?
        public static final boolean LEFT_MOTORS_INVERTED = false;
        public static final boolean RIGHT_MOTORS_INVERTED = true;
        
        // How fast should the robot drive?
        public static final double MAX_DRIVE_SPEED = 0.8; // 80% of full speed
        public static final double MAX_TURN_SPEED = 0.6; // 60% of full speed
        
        // For advanced auto-driving features
        public static final double WHEEL_DIAMETER_INCHES = 6.0;
        public static final double ENCODER_COUNTS_PER_REV = 42.0;
        public static final double GEAR_RATIO = 10.71; // Toughbox Mini 
    }

    /**
     * Controller Settings
     * For driver controllers and buttons
     */
    public static final class OI {
        // USB port numbers for controllers
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        
        // Deadband - ignores tiny joystick movements
        public static final double STICK_DEADBAND = 0.08;
        
        // Which joystick controls what?
        public static final int DRIVE_AXIS = 1; // Left Y-axis
        public static final int TURN_AXIS = 4;  // Right X-axis
        
        // Button assignments
        public static final int ARM_PICKUP_BUTTON = 1; // A button
        public static final int ARM_SCORE_BUTTON = 4;  // Y button
        public static final int INTAKE_BUTTON = 2;     // B button
        public static final int RELEASE_BUTTON = 3;    // X button
    }
    
    /**
     * Autonomous settings
     * For when the robot drives itself
     */
    public static final class Auto {
        // How long each auto routine runs
        public static final double DEFAULT_AUTO_TIME = 15.0; // Seconds
        
        // Simple auto routine distances
        public static final double DRIVE_FORWARD_DISTANCE = 36.0; // Inches
        public static final double DRIVE_FORWARD_SPEED = 0.5; // 50% power
    }
    
    /**
     * Electrical hardware ports
     * All electrical hardware connections in one place
     */
    public static final class ElectricalPorts {
        // Power Distribution Hub
        public static final int PDH_CAN_ID = 1;
        
        // Pneumatics Hub (if using pneumatics)
        public static final int PCM_CAN_ID = 2;
    }
}
