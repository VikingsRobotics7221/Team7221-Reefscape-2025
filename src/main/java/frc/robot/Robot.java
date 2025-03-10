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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Import commands
import frc.robot.commands.BallControlCommands;
import frc.robot.commands.BallTrackingCommand;
import frc.robot.commands.hook.HookCommands;

// Import autonomous routines
import frc.robot.commands.autonomous.StrategicBallHuntAuto;
import frc.robot.commands.autonomous.DefensiveHoardingAuto;
import frc.robot.commands.autonomous.ReefscapeAuto;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

// Import subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.HookSubsystem;

// Import utilities
import frc.robot.utils.MotorSafetyMonitor;
import frc.robot.utils.PerformanceDashboard;

/**
 * Robot Control System - Core control class for Team 7221's Reefscape 2025 robot
 * 
 * This class implements a command-based architecture that separates control logic from
 * hardware implementation. It coordinates all subsystems and provides the autonomous
 * and teleop mode behaviors.
 * 
 * Key features:
 * - Drawer slide arm with NEO + 16:1 gearbox for ball control
 * - Tank drive with optimized control for pushing power
 * - Vision-based tracking for autonomous ball acquisition
 * - Hook deployment system for endgame scoring
 */
public class Robot extends TimedRobot {

    // Autonomous command management
    private Command m_autonomousCommand;
    private final SendableChooser<Command> m_autonChooser = new SendableChooser<>();

    // Operating modes
    public static boolean manualDriveControl = true;
    private boolean m_turboModeEnabled = false;
    private boolean m_precisionModeEnabled = false;

    // Control interfaces
    public static final XboxController driveController = new XboxController(Constants.CONTROLLER_USB_PORT_ID);
    public static final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_USB_PORT_ID);

    // Subsystem initialization
    public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public static final BallArmSubsystem m_ballArmSubsystem = new BallArmSubsystem();
    public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final HookSubsystem m_hookSubsystem = new HookSubsystem();
    
    // State tracking
    private double m_goalAngle = 0.0;
    private int m_loopCounter = 0;

    @Override
    public void robotInit() {
        // System startup notification
        System.out.println("\n" +
            "=================================================\n" +
            "     TEAM 7221 - THE VIKINGS - REEFSCAPE 2025   \n" +
            "     ALL SYSTEMS ONLINE - PREPARED TO DOMINATE  \n" +
            "     NEO + 16:1 GEARBOX ARM SYSTEM ACTIVATED    \n" +
            "=================================================\n"
        );
        
        // Initialize monitoring systems
        MotorSafetyMonitor.initialize();
        PerformanceDashboard.initialize(false);  // Non-verbose mode
        
        // Register motors for safety monitoring
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getLeftFrontMotor(), "LeftFront");
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getRightFrontMotor(), "RightFront");
        // Commented out until these methods are confirmed to exist
        /* 
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getLeftBackMotor(), "LeftBack");
        MotorSafetyMonitor.registerMotor(m_driveSubsystem.getRightBackMotor(), "RightBack");
        */
        MotorSafetyMonitor.registerMotor(m_ballArmSubsystem.getExtensionMotor(), "BallArm");
        MotorSafetyMonitor.registerMotor(m_ballArmSubsystem.getGripperMotor(), "Gripper");
        MotorSafetyMonitor.registerMotor(m_hookSubsystem.getHookMotor(), "Hook");

        // Configure control bindings
        configureButtonBindings();
        
        // Configure autonomous modes
        m_autonChooser.setDefaultOption("No Action", new InstantCommand());
        m_autonChooser.addOption("Strategic Ball Hunt", new StrategicBallHuntAuto());
        m_autonChooser.addOption("Defensive Hoarding", new DefensiveHoardingAuto());
        m_autonChooser.addOption("Reefscape Optimized", new ReefscapeAuto());
        m_autonChooser.addOption("Drive Forward 1m", new Drivetrain_GyroStraight(1.0, 0.6));
        m_autonChooser.addOption("Test Square Pattern", createSquareTestAuto());
        
        // Register autonomous chooser with dashboard
        SmartDashboard.putData("Autonomous Mode", m_autonChooser);
        
        // Set up default commands for subsystems
        setDefaultCommands();
        
        // Initial setup for sensors and systems
        m_driveSubsystem.resetEncoders();
        m_driveSubsystem.zeroGyro();
        m_visionSubsystem.setPipeline(0); // Set to ball tracking by default
        
        // Dashboard information
        SmartDashboard.putString("Robot Name", "Team 7221 Viking Reefscape Robot");
        SmartDashboard.putString("Arm System", "NEO + 16:1 Gearbox Drawer Slide");
        SmartDashboard.putString("Status", "READY TO CONQUER THE FIELD!");
    }

    /**
     * Configure controller button bindings
     * Maps driver and operator controls to robot functions
     */
    private void configureButtonBindings() {
        // DRIVER CONTROLS - Primary movement and drive modes
        
        // A = Precision mode (50% power for fine control)
        new Trigger(() -> driveController.getAButton())
            .onTrue(new InstantCommand(() -> {
                m_precisionModeEnabled = true;
                m_driveSubsystem.enablePrecisionMode();
                System.out.println(">> PRECISION MODE ACTIVATED");
            }))
            .onFalse(new InstantCommand(() -> {
                m_precisionModeEnabled = false;
                m_driveSubsystem.disableDriveModes();
                System.out.println(">> PRECISION MODE DEACTIVATED");
            }));
            
        // B = Turbo mode (100% power for maximum speed)
        new Trigger(() -> driveController.getBButton())
            .onTrue(new InstantCommand(() -> {
                m_turboModeEnabled = true;
                m_driveSubsystem.enableTurboMode();
                System.out.println(">> TURBO MODE ACTIVATED");
            }))
            .onFalse(new InstantCommand(() -> {
                m_turboModeEnabled = false;
                m_driveSubsystem.disableDriveModes();
                System.out.println(">> TURBO MODE DEACTIVATED");
            }));
        
        // Quick turn buttons
        new Trigger(() -> driveController.getXButton())
            .onTrue(new Drivetrain_GyroTurn(-90)
                .beforeStarting(() -> System.out.println(">> QUICK TURN LEFT 90°")));
                
        new Trigger(() -> driveController.getYButton())
            .onTrue(new Drivetrain_GyroTurn(90)
                .beforeStarting(() -> System.out.println(">> QUICK TURN RIGHT 90°")));
        
        // Emergency stop - Both bumpers simultaneously
        new Trigger(() -> driveController.getLeftBumper() && driveController.getRightBumper())
            .onTrue(new InstantCommand(() -> {
                m_driveSubsystem.stop();
                m_ballArmSubsystem.emergencyStop();
                m_hookSubsystem.emergencyStop();
                System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
            }));
        
        // OPERATOR CONTROLS - Mechanism control
        
        // Ball arm control - A = Pickup position
        new Trigger(() -> operatorController.getAButton())
            .onTrue(new BallControlCommands.PickupSequence(m_ballArmSubsystem));
            
        // B = Home position
        new Trigger(() -> operatorController.getBButton())
            .onTrue(new InstantCommand(() -> m_ballArmSubsystem.homeArm()));
            
        // Y = Score position
        new Trigger(() -> operatorController.getYButton())
            .onTrue(new BallControlCommands.ScoreSequence(m_ballArmSubsystem));
            
        // X = Auto ball tracking
        new Trigger(() -> operatorController.getXButton())
            .onTrue(new BallTrackingCommand());
        
        // Gripper control with triggers
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
            
        new Trigger(() -> operatorController.getRightStickButton())
            .onTrue(new HookCommands.HookCycleCommand(m_hookSubsystem));
            
        // Emergency stop - Both bumpers simultaneously
        new Trigger(() -> operatorController.getLeftBumper() && operatorController.getRightBumper())
            .onTrue(new InstantCommand(() -> {
                m_ballArmSubsystem.emergencyStop();
                m_hookSubsystem.emergencyStop();
                System.out.println("!!! MECHANISM EMERGENCY STOP ACTIVATED !!!");
            }));
    }
    
    /**
     * Set up default commands for subsystems
     * These commands run when no other command is scheduled
     */
    private void setDefaultCommands() {
        // Drive subsystem - Default to arcade drive with controller input
        m_driveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    if (manualDriveControl) {
                        // Get joystick inputs
                        double throttle = -driveController.getLeftY();  // Forward/back
                        double turn = driveController.getRightX();      // Turning
                        
                        // Apply deadband to eliminate controller drift
                        throttle = Math.abs(throttle) < Constants.Controls.JOYSTICK_DEADBAND ? 0 : throttle;
                        turn = Math.abs(turn) < Constants.Controls.JOYSTICK_DEADBAND ? 0 : turn;
                        
                        // Drive the robot with the processed inputs
                        m_driveSubsystem.arcadeDrive(throttle, turn);
                    }
                },
                m_driveSubsystem
            )
        );
        
        // Ball arm subsystem - Manual control with operator joystick
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
     * Creates a test autonomous routine that drives in a square pattern
     * Useful for validating control performance
     * 
     * @return Command sequence for square pattern
     */
    private Command createSquareTestAuto() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> System.out.println(">> BEGINNING SQUARE TEST PATTERN")),
            
            // Forward 1m
            new Drivetrain_GyroStraight(1.0, 0.6),
            
            // Turn 90 degrees
            new Drivetrain_GyroTurn(90.0),
            
            // Forward 1m
            new Drivetrain_GyroStraight(1.0, 0.6),
            
            // Turn 90 degrees
            new Drivetrain_GyroTurn(90.0),
            
            // Forward 1m
            new Drivetrain_GyroStraight(1.0, 0.6),
            
            // Turn 90 degrees
            new Drivetrain_GyroTurn(90.0),
            
            // Forward 1m (completing square)
            new Drivetrain_GyroStraight(1.0, 0.6),
            
            // Return to original heading
            new Drivetrain_GyroTurn(90.0),
            
            new InstantCommand(() -> System.out.println(">> SQUARE PATTERN COMPLETE!"))
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        
        // Update dashboard with key information
        m_loopCounter++;
        if (m_loopCounter % 10 == 0) {  // Update every 10 loops to reduce CAN bus traffic
            SmartDashboard.putNumber("Gyro Angle", m_driveSubsystem.getGyroAngle());
            SmartDashboard.putNumber("Ball Arm Position", m_ballArmSubsystem.getArmPosition());
            SmartDashboard.putBoolean("Has Ball", m_ballArmSubsystem.hasBall());
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putBoolean("Vision Target", m_visionSubsystem.getHasTarget());
            SmartDashboard.putBoolean("Hook Extended", m_hookSubsystem.isExtended());
            SmartDashboard.putBoolean("Hook Retracted", m_hookSubsystem.isRetracted());
        }
        
        // Check for critical battery voltage
        if (m_loopCounter % 50 == 0) {
            double voltage = RobotController.getBatteryVoltage();
            if (voltage < Constants.Performance.BATTERY_WARNING_THRESHOLD) {
                System.out.println(">> WARNING: Low battery voltage: " + voltage + "V");
            }
        }
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_autonChooser.getSelected();
        
        // Get alliance information for path planning
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
        
        System.out.println("\n" +
            "=================================================\n" +
            "        AUTONOMOUS MODE INITIALIZED             \n" +
            "        ALLIANCE: " + (isRed ? "RED" : "BLUE") + "                          \n" +
            "=================================================\n"
        );
        
        // Reset critical sensors for autonomous
        m_driveSubsystem.resetEncoders();
        m_driveSubsystem.zeroGyro();
        
        // Execute the autonomous command
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // Command scheduler handles this through robotPeriodic()
    }

    @Override
    public void teleopInit() {
        System.out.println("\n" +
            "=================================================\n" +
            "        TELEOP MODE INITIALIZED                 \n" +
            "        DRIVER CONTROL ACTIVATED                \n" +
            "=================================================\n"
        );
        
        // Cancel autonomous command if running
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        
        // Reset drive modes to normal for teleop
        m_driveSubsystem.disableDriveModes();
        m_turboModeEnabled = false;
        m_precisionModeEnabled = false;
    }

    @Override
    public void teleopPeriodic() {
        // Command scheduler handles this through robotPeriodic()
    }

    @Override
    public void disabledInit() {
        System.out.println("\n" +
            "=================================================\n" +
            "        ROBOT DISABLED                          \n" +
            "        SYSTEMS ON STANDBY                      \n" +
            "=================================================\n"
        );
        
        // Log performance metrics
        System.out.println(">> PERFORMANCE SUMMARY:");
        
        // Commented until we verify these methods exist
        /* 
        System.out.println(">> MAX DRIVE CURRENT: " + m_driveSubsystem.getMaxCurrent() + "A");
        System.out.println(">> TOTAL DISTANCE: " + m_driveSubsystem.getTotalDistance() + "m");
        */
        System.out.println(">> BATTERY VOLTAGE: " + RobotController.getBatteryVoltage() + "V");
    }

    @Override
    public void disabledPeriodic() {
        // Nothing to do when disabled
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        System.out.println("\n>> TEST MODE INITIALIZED\n");
    }

    @Override
    public void testPeriodic() {
        // Test mode code here
    }

    @Override
    public void simulationInit() {
        // Simulation initialization code
    }

    @Override
    public void simulationPeriodic() {
        // Simulation periodic code
    }
}
