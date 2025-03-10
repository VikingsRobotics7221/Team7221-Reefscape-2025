// src/main/java/frc/robot/Robot.java
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

// Import subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.HookSubsystem;

// Import commands
import frc.robot.commands.BallControlCommands;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.hook.HookCommands;

// Import autonomous routines
import frc.robot.commands.autonomous.ReefscapeAuto;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;

/**
 * ┌───────────────────────────────────────────────────────────────┐
 * │              TEAM 7221 VIKING CONTROL SYSTEM                  │
 * │         THE BEATING HEART OF OUR REEFSCAPE ROBOT              │
 * └───────────────────────────────────────────────────────────────┘
 * 
 * This class orchestrates our robot's subsystems with performance-optimized
 * control logic, creating a responsive and dominant competitive machine.
 * 
 * SUBSYSTEM ARCHITECTURE:
 * ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
 * │  DRIVE   │    │   BALL   │    │  VISION  │    │   HOOK   │
 * │ SUBSYSTEM│    │    ARM   │    │ SUBSYSTEM│    │ SUBSYSTEM│
 * └────┬─────┘    └────┬─────┘    └────┬─────┘    └────┬─────┘
 *      │               │               │               │
 *      └───────────────┴───────────────┴───────────────┘
 *                           │
 *                     ┌─────┴─────┐
 *                     │  COMMAND  │
 *                     │ SCHEDULER │
 *                     └─────┬─────┘
 *                           │
 *                       ┌───┴───┐
 *                       │ ROBOT │
 *                       └───────┘
 */
public class Robot extends TimedRobot {

    // ========== CORE SYSTEM COMPONENTS ==========
    
    // Autonomous command management
    private Command m_autonomousCommand;
    private final SendableChooser<Command> m_autonChooser = new SendableChooser<>();

    // Operating modes
    private boolean m_turboModeEnabled = false;
    private boolean m_precisionModeEnabled = false;

    // Control interfaces
    public static final XboxController driveController = new XboxController(Constants.Controls.DRIVER_CONTROLLER_PORT);
    public static final XboxController operatorController = new XboxController(Constants.Controls.OPERATOR_CONTROLLER_PORT);

    // Subsystem initialization - THE FOUNDATION OF OUR ROBOT
    public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public static final BallArmSubsystem m_ballArmSubsystem = new BallArmSubsystem();
    public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final HookSubsystem m_hookSubsystem = new HookSubsystem();
    
    // State tracking
    private int m_loopCounter = 0;

    /**
     * Robot initialization - Called once when the robot first starts up
     * This is where we set up the entire robot control infrastructure
     */
    @Override
    public void robotInit() {
        System.out.println("\n" +
            "╔════════════════════════════════════════════════════╗\n" +
            "║   TEAM 7221 - THE VIKINGS - REEFSCAPE 2025         ║\n" +
            "║   INITIALIZING ROBOT CONTROL SYSTEMS               ║\n" +
            "╚════════════════════════════════════════════════════╝");
        
        // Configure control bindings for controllers
        configureButtonBindings();
        
        // Configure autonomous modes and selection
        configureAutonomousModes();
        
        // Set up default commands for subsystems
        setDefaultCommands();
        
        // Initial setup for sensors and systems
        m_driveSubsystem.resetEncoders();
        // m_driveSubsystem.zeroGyro(); // Uncomment if gyro is implemented
        m_visionSubsystem.setPipeline(0); // Set to ball tracking by default
        
        // Dashboard information
        SmartDashboard.putString("Robot Name", "Team 7221 Viking Reefscape Robot");
        SmartDashboard.putString("Status", "READY");
        
        System.out.println(">> ROBOT INITIALIZATION COMPLETE - READY FOR BATTLE!");
    }

    /**
     * Called periodically during all robot modes
     * This is the heartbeat of our control system
     */
    @Override
    public void robotPeriodic() {
        // Run the command scheduler - The orchestrator of all robot actions
        CommandScheduler.getInstance().run();
        
        // Update dashboard with key information
        updateDashboard();
        
        // System health monitoring (commented out for simplicity)
        // monitorSystemHealth();
    }

    /**
     * Called when the robot enters autonomous mode
     * Here we launch our pre-programmed battle strategy
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_autonChooser.getSelected();
        
        System.out.println("\n>> AUTONOMOUS MODE INITIALIZED");
        
        // Reset critical sensors for autonomous
        m_driveSubsystem.resetEncoders();
        // m_driveSubsystem.zeroGyro(); // Uncomment if gyro is implemented
        
        // Execute the autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * Called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        // Command scheduler handles this through robotPeriodic()
    }

    /**
     * Called when the robot enters teleop mode
     * This is where we shift to human control
     */
    @Override
    public void teleopInit() {
        System.out.println("\n>> TELEOP MODE INITIALIZED");
        System.out.println(">> DRIVER CONTROL ACTIVATED");
        
        // Cancel autonomous command if running
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        
        // Reset drive modes to normal for teleop
        m_driveSubsystem.arcadeDrive(0, 0); // Stop any movement
        m_turboModeEnabled = false;
        m_precisionModeEnabled = false;
    }

    /**
     * Called periodically during teleop
     */
    @Override
    public void teleopPeriodic() {
        // Command scheduler handles this through robotPeriodic()
    }

    /**
     * Called when the robot enters disabled mode
     */
    @Override
    public void disabledInit() {
        System.out.println("\n>> ROBOT DISABLED - SYSTEMS STANDBY");
    }

    /**
     * Called periodically while disabled
     */
    @Override
    public void disabledPeriodic() {
        // Nothing to do when disabled
    }

    /**
     * Called when entering test mode
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        System.out.println("\n>> TEST MODE INITIALIZED");
    }

    /**
     * Called periodically during test mode
     */
    @Override
    public void testPeriodic() {
        // Test mode code here
    }

    /**
     * Configure controller button bindings
     * This maps physical controller inputs to robot actions
     */
    private void configureButtonBindings() {
        // DRIVER CONTROLS - Primary movement and drive modes
        configureDriverControls();
        
        // OPERATOR CONTROLS - Mechanism control
        configureOperatorControls();
    }
    
    /**
     * Configure driver controller buttons
     * Primarily handles drivetrain control modes
     * 
     * ┌─────────────────────────────────────────┐
     * │        DRIVER CONTROLLER LAYOUT         │
     * ├─────────────┬───────────────────────────┤
     * │ Button A    │ Precision Mode (35%)      │
     * │ Button B    │ Turbo Mode (100%)         │
     * │ L+R Bumpers │ Emergency Stop            │
     * │ Left Stick  │ Forward/Backward          │
     * │ Right Stick │ Turning                   │
     * └─────────────┴───────────────────────────┘
     */
    private void configureDriverControls() {
        // A = Precision mode (35% power for fine control)
        new Trigger(() -> driveController.getAButton())
            .onTrue(new InstantCommand(() -> {
                m_precisionModeEnabled = true;
                System.out.println(">> Precision mode activated");
            }))
            .onFalse(new InstantCommand(() -> {
                m_precisionModeEnabled = false;
                System.out.println(">> Precision mode deactivated");
            }));
            
        // B = Turbo mode (100% power for maximum speed)
        new Trigger(() -> driveController.getBButton())
            .onTrue(new InstantCommand(() -> {
                m_turboModeEnabled = true;
                System.out.println(">> Turbo mode activated");
            }))
            .onFalse(new InstantCommand(() -> {
                m_turboModeEnabled = false;
                System.out.println(">> Turbo mode deactivated");
            }));
        
        // Emergency stop - Both bumpers simultaneously
        new Trigger(() -> driveController.getLeftBumper() && driveController.getRightBumper())
            .onTrue(new InstantCommand(() -> {
                m_driveSubsystem.stopMotors();
                m_hookSubsystem.emergencyStop();
                System.out.println(">> EMERGENCY STOP ACTIVATED");
            }));
    }
    
    /**
     * Configure operator controller buttons
     * Handles ball arm and hook mechanisms
     * 
     * ┌─────────────────────────────────────────┐
     * │      OPERATOR CONTROLLER LAYOUT         │
     * ├─────────────┬───────────────────────────┤
     * │ Button A    │ Ball Pickup Sequence      │
     * │ Button B    │ Arm Home Position         │
     * │ Button Y    │ Ball Scoring Sequence     │
     * │ Button X    │ Auto Ball Tracking        │
     * │ Left Stick  │ Manual Arm Control        │
     * │ R Trigger   │ Ball Intake (variable)    │
     * │ L Trigger   │ Ball Release (variable)   │
     * │ Back Button │ Extend Hook               │
     * │ Start Button│ Retract Hook              │
     * └─────────────┴───────────────────────────┘
     */
    private void configureOperatorControls() {
        // Ball arm control - A = Pickup position
        new Trigger(() -> operatorController.getAButton())
            .onTrue(new BallControlCommands.PickupSequence(m_ballArmSubsystem));
            
        // B = Home position
        new Trigger(() -> operatorController.getBButton())
            .onTrue(new InstantCommand(() -> {
                // Use setArmPosition with HOME_POSITION from Constants
                m_ballArmSubsystem.setArmPosition(Constants.BallArm.HOME_POSITION);
            }));
            
        // Y = Score position
        new Trigger(() -> operatorController.getYButton())
            .onTrue(new BallControlCommands.ScoreSequence(m_ballArmSubsystem));
            
        // X = Auto ball tracking
        new Trigger(() -> operatorController.getXButton())
            .onTrue(new BallTrackingCommand());
        
        // Gripper control with triggers - variable speed based on trigger position
        new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> 
                m_ballArmSubsystem.setGripper(operatorController.getRightTriggerAxis() * 
                Constants.BallArm.GRIPPER_INTAKE_SPEED)
            ))
            .onFalse(new InstantCommand(() -> m_ballArmSubsystem.setGripper(0)));
            
        new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> 
                m_ballArmSubsystem.setGripper(-operatorController.getLeftTriggerAxis() * 
                Constants.BallArm.GRIPPER_RELEASE_SPEED)
            ))
            .onFalse(new InstantCommand(() -> m_ballArmSubsystem.setGripper(0)));
            
        // Hook controls
        new Trigger(() -> operatorController.getBackButton())
            .onTrue(new HookCommands.ExtendHookCommand(m_hookSubsystem));
            
        new Trigger(() -> operatorController.getStartButton())
            .onTrue(new HookCommands.RetractHookCommand(m_hookSubsystem));
    }
    
    /**
     * Set up default commands for subsystems
     * These commands run when no other command is scheduled
     */
    private void setDefaultCommands() {
        // ========== DRIVE SUBSYSTEM DEFAULT COMMAND ==========
        // This implements arcade drive using the left stick for forward/backward
        // and the right stick for turning
        m_driveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get joystick inputs
                    double throttle = -driveController.getLeftY();  // Forward/back
                    double turn = driveController.getRightX();      // Turning
                    
                    // Apply deadband to eliminate controller drift
                    throttle = Math.abs(throttle) < Constants.Controls.JOYSTICK_DEADBAND ? 0 : throttle;
                    turn = Math.abs(turn) < Constants.Controls.JOYSTICK_DEADBAND ? 0 : turn;
                    
                    // Apply speed modifications based on current mode
                    if (m_precisionModeEnabled) {
                        throttle *= Constants.Drivetrain.DRIVE_PRECISION_SPEED;
                        turn *= Constants.Drivetrain.DRIVE_PRECISION_SPEED;
                    } else if (m_turboModeEnabled) {
                        throttle *= Constants.Drivetrain.DRIVE_TURBO_SPEED;
                        turn *= Constants.Drivetrain.DRIVE_TURBO_SPEED;
                    } else {
                        throttle *= Constants.Drivetrain.DRIVE_NORMAL_SPEED;
                        turn *= Constants.Drivetrain.DRIVE_NORMAL_SPEED;
                    }
                    
                    // Drive the robot with the processed inputs
                    m_driveSubsystem.arcadeDrive(throttle, turn);
                },
                m_driveSubsystem
            )
        );
        
        // ========== BALL ARM SUBSYSTEM DEFAULT COMMAND ==========
        // This allows manual control of the arm with the operator's left joystick
        m_ballArmSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Get joystick input for manual arm control
                    double armMove = -operatorController.getLeftY();
                    
                    // Apply deadband
                    if (Math.abs(armMove) < 0.1) {
                        armMove = 0;
                    } else {
                        // Scale to safe speed
                        armMove *= Constants.BallArm.MAX_SPEED;
                    }
                    
                    // Move the arm
                    m_ballArmSubsystem.moveArm(armMove);
                },
                m_ballArmSubsystem
            )
        );
    }
    
    /**
     * Configure autonomous modes available for selection
     * These are the pre-programmed routines that can run during autonomous period
     */
    private void configureAutonomousModes() {
        // Basic "do nothing" autonomous
        m_autonChooser.setDefaultOption("No Action", new InstantCommand());
        
        // Simple drive forward
        m_autonChooser.addOption("Drive Forward 1m", new Drivetrain_GyroStraight(1.0, 0.6));
        
        // Game-specific autonomous routine
        m_autonChooser.addOption("Reefscape Auto", new ReefscapeAuto());
        
        // Add more autonomous options here
        // m_autonChooser.addOption("Ball Collection Auto", new StrategicBallHuntAuto());
        // m_autonChooser.addOption("Defensive Hoarding Auto", new DefensiveHoardingAuto());
        
        // Register autonomous chooser with dashboard
        SmartDashboard.putData("Autonomous Mode", m_autonChooser);
    }
    
    /**
     * Update dashboard with key robot information
     * Displays status of all major subsystems
     */
    private void updateDashboard() {
        m_loopCounter++;
        if (m_loopCounter % 10 == 0) {  // Update every 10 loops to reduce CAN bus traffic
            SmartDashboard.putNumber("Ball Arm Position", m_ballArmSubsystem.getArmPosition());
            SmartDashboard.putBoolean("Has Ball", m_ballArmSubsystem.hasBall());
            SmartDashboard.putBoolean("Vision Target", m_visionSubsystem.getHasTarget());
            SmartDashboard.putBoolean("Hook Extended", m_hookSubsystem.isExtended());
            SmartDashboard.putBoolean("Hook Retracted", m_hookSubsystem.isRetracted());
        }
    }
    
    // ========== ADVANCED MONITORING SYSTEMS (COMMENTED OUT) ==========
    
    // /**
    //  * Monitor system health for critical issues
    //  * Checks battery voltage, motor currents, and sensor status
    //  */
    // private void monitorSystemHealth() {
    //     // Check battery voltage
    //     double currentVoltage = RobotController.getBatteryVoltage();
    //     if (currentVoltage < Constants.Performance.BATTERY_WARNING_THRESHOLD) {
    //         System.out.println("WARNING: Low battery voltage: " + currentVoltage + "V");
    //     }
    // 
    //     // Monitor motor temperatures and currents
    //     // MotorSafetyMonitor.updateAll();
    // }
    
    // /**
    //  * Create a square test autonomous routine
    //  * Useful for validating drivetrain performance
    //  */
    // private Command createSquareTestAuto() {
    //     return new SequentialCommandGroup(
    //         new Drivetrain_GyroStraight(1.0, 0.6),  // Forward 1m
    //         new Drivetrain_GyroTurn(90.0),          // Turn 90 degrees
    //         new Drivetrain_GyroStraight(1.0, 0.6),  // Forward 1m
    //         new Drivetrain_GyroTurn(90.0),          // Turn 90 degrees
    //         new Drivetrain_GyroStraight(1.0, 0.6),  // Forward 1m
    //         new Drivetrain_GyroTurn(90.0),          // Turn 90 degrees
    //         new Drivetrain_GyroStraight(1.0, 0.6),  // Forward 1m
    //         new Drivetrain_GyroTurn(90.0)           // Return to original heading
    //     );
    // }
}
