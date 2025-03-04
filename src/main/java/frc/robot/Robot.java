// Updated Robot.java with Ball Control System integration
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.autonomous.example_basic_auto.Drive1MeterAuto;
import frc.robot.commands.autonomous.example_basic_auto.SquareAutonomous;
import frc.robot.commands.BallControlCommands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.BallArmSubsystem;
import frc.robot.commands.TargetAndCollectCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.commands.hook.HookCommands;
import frc.robot.commands.BallTargetingCommand;

public class Robot extends TimedRobot {

    Command m_autonomousCommand;
SendableChooser<Command> autonChooser = new SendableChooser<Command>();
autonChooser.addOption("Ball Collection Auto", new BallCollectionAuto());

    public static boolean manualDriveControl = true;

    // Controllers
    public static XboxController driveController = new XboxController(Constants.CONTROLLER_USB_PORT_ID);
    public static XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_USB_PORT_ID);

    // Subsystems
    public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    public static final BallArmSubsystem m_ballArmSubsystem = new BallArmSubsystem();
    public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    public static final HookSubsystem m_hookSubsystem = new HookSubsystem();
    
    double goalAngle;
    boolean turboModeEnabled = false;
    boolean precisionModeEnabled = false;

    @Override
    public void robotInit() {
        configureButtonBindings();
   // X Button - Auto Ball Targeting
new Trigger(() -> operatorController.getXButton())
    .onTrue(new BallTargetingCommand());
    
   // Left Bumper + Right Bumper together - Emergency stop all systems
new Trigger(() -> operatorController.getLeftBumper() && operatorController.getRightBumper())
    .onTrue(new InstantCommand(() -> {
        m_ballArmSubsystem.emergencyStop();
        m_hookSubsystem.emergencyStop();
        m_driveSubsystem.stop();
        System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
    
   // Back Button - Extend hook
new Trigger(() -> operatorController.getBackButton())
    .onTrue(new HookCommands.ExtendHookCommand(m_hookSubsystem));
    
   // Start Button - Retract hook
new Trigger(() -> operatorController.getStartButton())
    .onTrue(new HookCommands.RetractHookCommand(m_hookSubsystem));
    
   // Right Stick Button - Run full hook cycle
new Trigger(() -> operatorController.getRightStickButton())
    .onTrue(new HookCommands.HookCycleCommand(m_hookSubsystem));
   // POV Up - Switch vision to ball tracking
new Trigger(() -> operatorController.getPOV() == 0)
    .onTrue(new InstantCommand(() -> m_visionSubsystem.setPipeline(0)));

   // POV Down - Switch vision to AprilTag
new Trigger(() -> operatorController.getPOV() == 180)
    .onTrue(new InstantCommand(() -> m_visionSubsystem.setPipeline(1)));

   // POV Right - Toggle driver camera mode
new Trigger(() -> operatorController.getPOV() == 90)
    .onTrue(new InstantCommand(() -> {
        boolean current = m_visionSubsystem.getCurrentPipeline() == 2;
        m_visionSubsystem.setDriverMode(!current);
    }));
        
        // Setup autonomous chooser
        autonChooser.setDefaultOption("Do Nothing", new InstantCommand());
        autonChooser.addOption("Drive 1 Meter", new Drive1MeterAuto());
        autonChooser.addOption("Square Autonomous", new SquareAutonomous());
        autonChooser.addOption("Ball Pickup Auto", createBallPickupAuto());
        SmartDashboard.putData("Auto Mode", autonChooser);

        // Zero gyro and reset encoders
        m_driveSubsystem.zeroGyro();
        m_driveSubsystem.resetEncoders();
        
        // Set default commands
        setDefaultCommands();
        
        // Dashboard info
        SmartDashboard.putString("Robot Name", "Team 7221 Reefscape Robot");
        SmartDashboard.putString("Status", "Ready to Dominate!");
    }

    private void setDefaultCommands() {
        // Set default drive command to arcade drive with controller inputs
        m_driveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> {
                    if (manualDriveControl) {
                        double throttle = -driveController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
                        double turn = -driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS);
                        
                        // Apply deadband to prevent drift
                        throttle = Math.abs(throttle) < 0.05 ? 0 : throttle;
                        turn = Math.abs(turn) < 0.05 ? 0 : turn;
                        
                        // Apply speed limiters
                        if (turboModeEnabled) {
                            // No limiting in turbo mode
                        } else if (precisionModeEnabled) {
                            throttle *= 0.4; // 40% speed in precision mode
                            turn *= 0.3;     // 30% turning in precision mode
                        } else {
                            throttle *= 0.8; // 80% speed in normal mode
                            turn *= 0.6;     // 60% turning in normal mode
                        }
                        
                        m_driveSubsystem.arcadeDrive(throttle, turn);
                    }
                },
                m_driveSubsystem
            )
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        
        // Log gyro data
        SmartDashboard.putNumber("Gyroscope Pitch", m_driveSubsystem.getPitch());
        SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
        SmartDashboard.putNumber("Gyroscope Roll", m_driveSubsystem.getRoll());
        
        // Log battery voltage
        SmartDashboard.putNumber("Battery Voltage", DriverStation.getBatteryVoltage());
    }

    @Override
    public void disabledInit() {
        System.out.println("⚠️ ROBOT DISABLED");
    }

    @Override
    public void autonomousInit() {
        System.out.println("🤖 AUTONOMOUS MODE STARTED");

        m_autonomousCommand = autonChooser.getSelected();
        
        m_driveSubsystem.zeroGyro();
        m_driveSubsystem.resetEncoders();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        System.out.println("👾 TELEOP MODE STARTED");

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_driveSubsystem.zeroGyro();
        m_driveSubsystem.resetEncoders();
        
        // Get alliance color for LEDs
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                System.out.println("🔴 Red Alliance");
            } else {
                System.out.println("🔵 Blue Alliance");
            }
        }

        goalAngle = m_driveSubsystem.getGyroAngle();
        
        // Reset drive modes
        turboModeEnabled = false;
        precisionModeEnabled = false;
        m_driveSubsystem.disableDriveModes();
    }

    @Override
    public void teleopPeriodic() {
        // Controller input logging
        SmartDashboard.putNumber("Left Y", driveController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
        SmartDashboard.putNumber("Right X", driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
        
        // Update goal angle for gyro assistance
        if (Math.abs(driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS)) > 0.1) {
            goalAngle = m_driveSubsystem.getGyroAngle();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        System.out.println("🧪 TEST MODE STARTED");
    }

    private void configureButtonBindings() {
        // ==== DRIVER CONTROLS ====
        // Turbo mode (right bumper)
        new Trigger(() -> driveController.getRightBumper())
            .onTrue(new InstantCommand(() -> {
                turboModeEnabled = true;
                precisionModeEnabled = false;
                m_driveSubsystem.enableTurboMode();
                System.out.println("🔥 TURBO MODE ACTIVATED");
            }))
            .onFalse(new InstantCommand(() -> {
                turboModeEnabled = false;
                m_driveSubsystem.disableDriveModes();
                System.out.println("🔥 TURBO MODE DEACTIVATED");
            }));
        
        // Precision mode (left bumper)
        new Trigger(() -> driveController.getLeftBumper())
            .onTrue(new InstantCommand(() -> {
                precisionModeEnabled = true;
                turboModeEnabled = false;
                m_driveSubsystem.enablePrecisionMode();
                System.out.println("🔍 PRECISION MODE ACTIVATED");
            }))
            .onFalse(new InstantCommand(() -> {
                precisionModeEnabled = false;
                m_driveSubsystem.disableDriveModes();
                System.out.println("🔍 PRECISION MODE DEACTIVATED");
            }));
        
        // ==== OPERATOR CONTROLS ====
        // A Button - Pickup sequence
        new Trigger(() -> operatorController.getAButton())
            .onTrue(new BallControlCommands.PickupSequence(m_ballArmSubsystem));
        
        // Y Button - Score sequence
        new Trigger(() -> operatorController.getYButton())
            .onTrue(new BallControlCommands.ScoreSequence(m_ballArmSubsystem));
            
        // B Button - Home position
        new Trigger(() -> operatorController.getBButton())
            .onTrue(new InstantCommand(() -> m_ballArmSubsystem.homeArm()));
            
        // Manual arm control - Left joystick Y axis
        m_ballArmSubsystem.setDefaultCommand(
            new RunCommand(() -> {
                double armSpeed = -operatorController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
                
                // Apply deadband and scale
                if (Math.abs(armSpeed) < 0.1) {
                    armSpeed = 0;
                } else {
                    // Scale to safe speed
                    armSpeed *= Constants.BALL_ARM_MAX_SPEED;
                }
                
                m_ballArmSubsystem.moveArm(armSpeed);
            }, m_ballArmSubsystem)
        );
        
        // Manual gripper control
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
    }
    
    // Create a custom autonomous routine
    private Command createBallPickupAuto() {
        return new SequentialCommandGroup(
            // Move forward to approach ball
            new RunCommand(() -> m_driveSubsystem.arcadeDrive(0.3, 0), m_driveSubsystem)
                .withTimeout(1.5),
                
            // Stop driving
            new InstantCommand(() -> m_driveSubsystem.stop()),
            
            // Run pickup sequence
            new BallControlCommands.PickupSequence(m_ballArmSubsystem),
            
            // Turn around (180 degrees)
            new RunCommand(() -> m_driveSubsystem.arcadeDrive(0, 0.6), m_driveSubsystem)
                .withTimeout(2.0),
                
            // Drive back to starting position
            new RunCommand(() -> m_driveSubsystem.arcadeDrive(0.3, 0), m_driveSubsystem)
                .withTimeout(1.5),
                
            // Stop and score
            new InstantCommand(() -> m_driveSubsystem.stop()),
            new BallControlCommands.ScoreSequence(m_ballArmSubsystem)
        );
    }
}
