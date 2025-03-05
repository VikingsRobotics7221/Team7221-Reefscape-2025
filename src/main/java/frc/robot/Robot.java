// src/main/java/frc/robot/Robot.java
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Import our commands
import frc.robot.commands.BallControlCommands;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.hook.HookCommands;

// Import our autonomous routines
import frc.robot.commands.autonomous.StrategicBallHuntAuto;
import frc.robot.commands.autonomous.DefensiveHoardingAuto;
import frc.robot.commands.autonomous.ReefscapeAuto;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

// Import our subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.HookSubsystem;

// Import our utility classes
import frc.robot.utils.MotorSafetyMonitor;
import frc.robot.utils.PerformanceDashboard;

/**
 * THE MIND OF TEAM 7221's REEFSCAPE ROBOT!!
 * 
 * This class is the CORE of our robot code - it initializes all subsystems,
 * handles command scheduling, and manages driver inputs. Our 16:1 drive ratio
 * combined with precise ball control will DOMINATE the competition!
 * 
 * coded by paysean - Team 7221 Viking Code Warrior
 * Last updated: March 2025
 */
public class Robot extends TimedRobot {

    // Autonomous command tracking
    private Command m_autonomousCommand;
    private final SendableChooser<Command> m_autonChooser = new SendableChooser<>();

    // Manual drive override flag
    public static boolean manualDriveControl = true;

    // CONTROLLERS - THE DRIVER INTERFACE!!
    public static final XboxController driveController = new XboxController(Constants.CONTROLLER_USB_PORT_ID);
    public static final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_USB_PORT_ID);

    // SUBSYSTEMS - THE MUSCLES AND ORGANS!!
    public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public static final BallArmSubsystem m_ballArmSubsystem = new BallArmSubsystem();
    public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final HookSubsystem m_hookSubsystem = new HookSubsystem();
    
    // State tracking variables
    private double m_goalAngle = 0.0;
    private boolean m_turboModeEnabled = false;
    private boolean m_precisionModeEnabled = false;
    private int m_loopCounter = 0;

    @Override
    public void robotInit() {
        // Boot up message - GET HYPED!!
        System.out.println("");
        System.out.println("╔════════════════════════════════════════════════════════╗");
        System.out.println("║     TEAM 7221 - THE VIKINGS - REEFSCAPE 2025 ROBOT     ║");
        System.out.println("║ SYSTEMS ONLINE - BALL TRACKING ACTIVE - 16:1 OPTIMIZED ║");
        System.out.println("║           PREPARE FOR TOTAL DOMINATION!!!              ║");
        System.out.println("╚════════════════════════════════════════════════════════╝");
        System.out.println("");
        
        // Initialize our utility monitoring systems
        MotorSafetyMonitor.initialize();
        PerformanceDashboard.initialize(false);  // Start in non-verbose mode
        
        // Register all motors for safety monitoring
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getLeftFrontMotor(), "LeftFront");
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getRightFrontMotor(), "RightFront");
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getLeftBackMotor(), "LeftBack");
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getRightBackMotor(), "RightBack");
        MotorSafetyMonitor.registerMotor(m_ballArmSubsystem.getExtensionMotor(), "BallArm");
        MotorSafetyMonitor.registerMotor(m_ballArmSubsystem.getGripperMotor(), "Gripper");
        MotorSafetyMonitor.registerMotor(m_hookSubsystem.getHookMotor(), "Hook");

        // Configure button bindings for teleop
        configureButtonBindings();
        
        // Setup autonomous chooser with our ULTIMATE autonomous routines
        m_autonChooser.setDefaultOption("Do Nothing", new InstantCommand());
        m_autonChooser.addOption("Strategic Ball Hunt (THE BEST!)", new StrategicBallHuntAuto());
        m_autonChooser.addOption("Defensive Hoarding", new DefensiveHoardingAuto());
        m_autonChooser.addOption("Standard Reefscape Auto", new ReefscapeAuto());
        m_autonChooser.addOption("Simple Drive Forward", createSimpleDriveForward());
        m_autonChooser.addOption("Test Square Pattern", createSquareTestPattern());
        
        SmartDashboard.putData("AUTO MODE SELECTOR", m_autonChooser);

        // Zero gyro and reset encoders for a clean slate
        m_driveSubsystem.zeroGyro();
        m_driveSubsystem.resetEncoders();
        
        // Set up default commands
        setDefaultCommands();
        
        // Dashboard info - SHOW OFF OUR AWESOME ROBOT!
        SmartDashboard.putString("Robot Name", "Team 7221 Viking Reefscape Dominator");
        SmartDashboard.putString("Drive Ratio", "16:1 MAXIMUM TORQUE!");
        SmartDashboard.putString("Status", "READY TO DESTROY COMPETITION!");
    }

    /**
     * Set up the default commands for each subsystem
     * These run when no other commands are scheduled for that subsystem
     */
    private void setDefaultCommands() {
        // Set default drive command to arcade drive with controller inputs
        m_driveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    if (manualDriveControl) {
                        // Start performance timing for drive loop
                        PerformanceDashboard.startTimer("DriveLoop");
                        
                        // Get joystick inputs
                        double throttle = -driveController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
                        double turn = -driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS);
                        
                        // Apply deadband to prevent drift
                        throttle = Math.abs(throttle) < Constants.JOYSTICK_DEADBAND ? 0 : throttle;
                        turn = Math.abs(turn) < Constants.JOYSTICK_DEADBAND ? 0 : turn;
                        
                        // Apply speed limiters based on drive mode
                        if (m_turboModeEnabled) {
                            // TURBO MODE = FULL POWER!!
                            // No limiting in turbo mode - use with caution!
                        } else if (m_precisionModeEnabled) {
                            throttle *= Constants.DRIVE_PRECISION_SPEED; // Reduced speed in precision mode
                            turn *= Constants.DRIVE_PRECISION_SPEED * 0.8; // Even slower turning for precision
                        } else {
                            throttle *= Constants.DRIVE_NORMAL_SPEED; // Normal mode = 85% power
                            turn *= Constants.DRIVE_NORMAL_SPEED * 0.7; // Slightly reduced turning
                        }
                        
                        // Send commands to the drivetrain
                        m_driveSubsystem.arcadeDrive(throttle, turn);
                        
                        // End performance timing
                        PerformanceDashboard.stopTimer("DriveLoop");
                    }
                },
                m_driveSubsystem
            )
        );
        
        // Set manual arm control as default for ball arm
        m_ballArmSubsystem.setDefaultCommand(
            new RunCommand(() -> {
                // Only use operator's left joystick input if it's significant
                double armSpeed = -operatorController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
                
                // Apply deadband and scale
                if (Math.abs(armSpeed) < 0.1) {
                    armSpeed = 0;
                } else {
                    // Scale to safe speed limit
                    armSpeed *= Constants.BALL_ARM_MAX_SPEED;
                }
                
                m_ballArmSubsystem.moveArm(armSpeed);
            }, m_ballArmSubsystem)
        );
    }

    @Override
    public void robotPeriodic() {
        // Run the command scheduler - THE HEARTBEAT OF OUR ROBOT!
        CommandScheduler.getInstance().run();
        
        // Start performance tracking for this loop
        PerformanceDashboard.startLoopTiming();
        
        // Increment loop counter (for periodic tasks)
        m_loopCounter++;
        
        // Update motor safety status every 10 loops (reduces CPU usage)
        if (m_loopCounter % 10 == 0) {
            MotorSafetyMonitor.updateAll();
        }
        
        // Log sensor data to SmartDashboard (every 5 loops)
        if (m_loopCounter % 5 == 0) {
            // Gyro data
            SmartDashboard.putNumber("Gyro Angle", m_driveSubsystem.getGyroAngle());
            SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
            
            // Battery voltage - CRITICAL FOR COMP!
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            
            // Distance traveled
            SmartDashboard.putNumber("Distance Traveled", m_driveSubsystem.getTotalDistance());
            
            // Camera status
            SmartDashboard.putBoolean("Target Visible", m_visionSubsystem.getHasTarget());
            if (m_visionSubsystem.getHasTarget()) {
                SmartDashboard.putNumber("Target Distance", m_visionSubsystem.getTargetDistance());
            }
        }
        
        // Generate performance report once per second
        if (m_loopCounter % 50 == 0) {
            // Calculate CPU load and other performance metrics
            double loopTimeMs = PerformanceDashboard.stopTimer("RobotPeriodic") * 1000.0;
            SmartDashboard.putNumber("CPU Loop Time (ms)", loopTimeMs);
        }
        
        // End performance tracking
        PerformanceDashboard.endLoopTiming();
    }

    @Override
    public void disabledInit() {
        System.out.println(">> ROBOT DISABLED");
        System.out.println(">> Performance Report:");
        System.out.println(PerformanceDashboard.getPerformanceReport());
    }

    @Override
    public void autonomousInit() {
        System.out.println("");
        System.out.println("╔═══════════════════════════════════════╗");
        System.out.println("║       AUTONOMOUS MODE ACTIVATED       ║");
        System.out.println("║ TEAM 7221 TAKING CONTROL OF THE FIELD ║");
        System.out.println("╚═══════════════════════════════════════╝");
        System.out.println("");

        // Get selected autonomous routine
        m_autonomousCommand = m_autonChooser.getSelected();
        
        // Reset all sensors for accurate autonomous
        m_driveSubsystem.zeroGyro();
        m_driveSubsystem.resetEncoders();
        
        // Reset performance metrics for monitoring
        PerformanceDashboard.reset();

        // Start the autonomous command if we have one!
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        System.out.println("");
        System.out.println("╔═══════════════════════════════════════╗");
        System.out.println("║         TELEOP MODE ACTIVATED         ║");
        System.out.println("║ DRIVER CONTROL ENABLED - LET'S DOMINATE!║");
        System.out.println("╚═══════════════════════════════════════╝");
        System.out.println("");

        // Cancel autonomous when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // Reset sensors for clean teleop control
        m_driveSubsystem.zeroGyro();
        m_driveSubsystem.resetEncoders();
        
        // Get alliance color for dashboard display
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                System.out.println(">> RED ALLIANCE - TARGET BLUE BALLS FIRST");
                SmartDashboard.putString("Alliance", "RED");
            } else {
                System.out.println(">> BLUE ALLIANCE - TARGET RED BALLS FIRST");
                SmartDashboard.putString("Alliance", "BLUE");
            }
        }

        // Store initial angle for drift correction
        m_goalAngle = m_driveSubsystem.getGyroAngle();
        
        // Reset drive modes
        m_turboModeEnabled = false;
        m_precisionModeEnabled = false;
        m_driveSubsystem.disableDriveModes();
        
        // Enable manual drive control
        manualDriveControl = true;
        
        // Reset performance tracking for teleop session
        PerformanceDashboard.reset();
    }

    @Override
    public void teleopPeriodic() {
        // Monitor controller inputs for debugging
        SmartDashboard.putNumber("Left Y", driveController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
        SmartDashboard.putNumber("Right X", driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
        
        // Update drive goal angle for assisted driving when turning
        if (Math.abs(driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS)) > 0.1) {
            m_goalAngle = m_driveSubsystem.getGyroAngle();
        }
    }

    @Override
    public void testInit() {
        // Cancel all commands for safety when entering test mode
        CommandScheduler.getInstance().cancelAll();
        System.out.println(">> TEST MODE STARTED");
        
        // Reset performance tracking for test session
        PerformanceDashboard.reset();
    }

    /**
     * Configure button bindings for teleop control
     * ALL THE AWESOME CONTROLS THAT MAKE OUR ROBOT SHINE!
     */
    private void configureButtonBindings() {
        // ===== DRIVER CONTROLS (MOVEMENT) =====
        
        // TURBO MODE - Right bumper for MAXIMUM POWER!
        new Trigger(() -> driveController.getRightBumper())
            .onTrue(new InstantCommand(() -> {
                m_turboModeEnabled = true;
                m_precisionModeEnabled = false;
                m_driveSubsystem.enableTurboMode();
                System.out.println(">> TURBO MODE ACTIVATED! MAXIMUM POWER!!!");
            }))
            .onFalse(new InstantCommand(() -> {
                m_turboModeEnabled = false;
                m_driveSubsystem.disableDriveModes();
                System.out.println(">> TURBO MODE DEACTIVATED");
            }));
        
        // PRECISION MODE - Left bumper for fine control
        new Trigger(() -> driveController.getLeftBumper())
            .onTrue(new InstantCommand(() -> {
                m_precisionModeEnabled = true;
                m_turboModeEnabled = false;
                m_driveSubsystem.enablePrecisionMode();
                System.out.println(">> PRECISION MODE ACTIVATED! Fine control enabled.");
            }))
            .onFalse(new InstantCommand(() -> {
                m_precisionModeEnabled = false;
                m_driveSubsystem.disableDriveModes();
                System.out.println(">> PRECISION MODE DEACTIVATED");
            }));
        
        // Quick turn buttons (for fast 90° turns)
        new Trigger(() -> driveController.getYButton())
            .onTrue(new Drivetrain_GyroTurn(90)
                .beforeStarting(() -> System.out.println(">> QUICK TURN RIGHT 90°")));
        
        new Trigger(() -> driveController.getXButton())
            .onTrue(new Drivetrain_GyroTurn(-90)
                .beforeStarting(() -> System.out.println(">> QUICK TURN LEFT 90°")));
        
        // Emergency stop - Back + Start buttons together
        new Trigger(() -> driveController.getBackButton() && driveController.getStartButton())
            .onTrue(new InstantCommand(() -> {
                m_driveSubsystem.stop();
                m_ballArmSubsystem.emergencyStop();
                m_hookSubsystem.emergencyStop();
                System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
            }));
        
        // ===== OPERATOR CONTROLS (MECHANISMS) =====
        
        // BALL ARM CONTROLS
        
        // A Button - Ball Pickup Sequence
        new Trigger(() -> operatorController.getAButton())
            .onTrue(new BallControlCommands.PickupSequence(m_ballArmSubsystem));
        
        // Y Button - Ball Score Sequence
        new Trigger(() -> operatorController.getYButton())
            .onTrue(new BallControlCommands.ScoreSequence(m_ballArmSubsystem));
        
        // B Button - Return arm to home position
        new Trigger(() -> operatorController.getBButton())
            .onTrue(new InstantCommand(() -> m_ballArmSubsystem.homeArm()));
        
        // X Button - Auto Ball Tracking (SUPER COOL FEATURE!!!)
        new Trigger(() -> operatorController.getXButton())
            .onTrue(new BallTrackingCommand());
        
        // Manual Gripper Control with Triggers
        new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> 
                m_ballArmSubsystem.setGripper(operatorController.getRightTriggerAxis() * 
                    Constants.BALL_GRIPPER_INTAKE_SPEED)
            ));
        
        new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> 
                m_ballArmSubsystem.setGripper(-operatorController.getLeftTriggerAxis() * 
                    Constants.BALL_GRIPPER_RELEASE_SPEED)
            ));
        
        // HOOK SYSTEM CONTROLS
        
        // Back Button - Extend hook
        new Trigger(() -> operatorController.getBackButton())
            .onTrue(new HookCommands.ExtendHookCommand(m_hookSubsystem));
        
        // Start Button - Retract hook
        new Trigger(() -> operatorController.getStartButton())
            .onTrue(new HookCommands.RetractHookCommand(m_hookSubsystem));
        
        // Right Stick Button - Run full hook cycle (extend, wait, retract)
        new Trigger(() -> operatorController.getRightStickButton())
            .onTrue(new HookCommands.HookCycleCommand(m_hookSubsystem));
        
        // VISION SYSTEM CONTROLS
        
        // POV Up - Switch to ball tracking pipeline
        new Trigger(() -> operatorController.getPOV() == 0)
            .onTrue(new InstantCommand(() -> m_visionSubsystem.setPipeline(0)));
        
        // POV Down - Switch to AprilTag pipeline
        new Trigger(() -> operatorController.getPOV() == 180)
            .onTrue(new InstantCommand(() -> m_visionSubsystem.setPipeline(1)));
        
        // POV Right - Toggle driver camera mode
        new Trigger(() -> operatorController.getPOV() == 90)
            .onTrue(new InstantCommand(() -> {
                boolean current = m_visionSubsystem.getCurrentPipeline() == 2;
                m_visionSubsystem.setDriverMode(!current);
            }));
        
        // EMERGENCY CONTROLS
        
        // Left Bumper + Right Bumper together - Emergency stop all systems
        new Trigger(() -> operatorController.getLeftBumper() && operatorController.getRightBumper())
            .onTrue(new InstantCommand(() -> {
                m_ballArmSubsystem.emergencyStop();
                m_hookSubsystem.emergencyStop();
                m_driveSubsystem.stop();
                System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
            }));
    }
    
    /**
     * Create a simple drive forward autonomous routine
     * Good for testing and quick deployment!
     * 
     * @return Command for driving forward 1 meter
     */
    private Command createSimpleDriveForward() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                System.out.println(">> Starting Simple Drive Forward");
                m_driveSubsystem.resetEncoders();
            }),
            new Drivetrain_GyroStraight(1.0, 0.4),
            new InstantCommand(() -> System.out.println(">> Simple Drive Complete!"))
        );
    }
    
    /**
     * Create a square test pattern autonomous routine
     * Great for testing turning accuracy and consistency!
     * 
     * @return Command for driving in a square pattern
     */
    private Command createSquareTestPattern() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                System.out.println(">> Starting Square Test Pattern");
                m_driveSubsystem.resetEncoders();
            }),
            // Drive in a square pattern: forward, turn, forward, turn, etc.
            new Drivetrain_GyroStraight(1.0, 0.4),
            new Drivetrain_GyroTurn(90),
            new Drivetrain_GyroStraight(1.0, 0.4),
            new Drivetrain_GyroTurn(90),
            new Drivetrain_GyroStraight(1.0, 0.4),
            new Drivetrain_GyroTurn(90),
            new Drivetrain_GyroStraight(1.0, 0.4),
            new Drivetrain_GyroTurn(90),
            new InstantCommand(() -> System.out.println(">> Square Test Complete!"))
        );
    }
}
