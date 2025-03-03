// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.autonomous.example_basic_auto.Drive1MeterAuto;
import frc.robot.commands.autonomous.example_basic_auto.SquareAutonomous;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Command m_autonomousCommand;
	SendableChooser<Command> autonChooser = new SendableChooser<Command>(); // Create a chooser to select an autonomous command

  public static boolean manualDriveControl = true;

  public static XboxController driveController = new XboxController(0);

  //public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  
  double goalAngle;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {


    configureButtonBindings(); // Bind our commands to physical buttons on a controller

    // Add our Autonomous Routines to the chooser //
		autonChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autonChooser.addOption("Drive 1 Meter", new Drive1MeterAuto());
    autonChooser.addOption("Square Autonomous", new SquareAutonomous());
		SmartDashboard.putData("Auto Mode", autonChooser);

    // Zero the gyroscope and reset the drive encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Gyroscope Pitch", m_driveSubsystem.getPitch());
    SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
    SmartDashboard.putNumber("Gyroscope Roll", m_driveSubsystem.getRoll());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("ROBOT DISABLED");
  }

  /** This function is called continuously after the robot enters Disabled mode. */
  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    System.out.println("AUTONOMOUS MODE STARTED");

    m_autonomousCommand = new RunCommand(
      () -> m_driveSubsystem.driveCartesian(0.2, 0, 0), m_driveSubsystem).withTimeout(1.5);
    
    // Zero the gyrodcope and reset the drive encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Set the LED pattern for autonomous mode

    // Set Elevator/End Effector inital preset
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    System.out.println("TELEOP MODE STARTED");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this if statement or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyroscope and reset the drive encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // Set the LED pattern for teleop mode

      }
      if (ally.get() == Alliance.Blue) {
      }
    }

    goalAngle = m_driveSubsystem.getGyroAngle();

    // // Set Elevator/End Effector inital preset
    // m_CoralElevatorSubsystem.climbNeutral();
    // m_CoralElevatorSubsystem.armInitial();

    // m_intakeSubsystem.setDefaultCommand(new IntakeManualControl());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Log controller inputs to SmartDashboard
    // SmartDashboard.putNumber("Controller: Right Trigger", controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS));
    // SmartDashboard.putNumber("Controller: Left Trigger", controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS));
    // SmartDashboard.putBoolean("Controller: Right Bumper", controller.getRawButton(Constants.RIGHT_BUMPER));
    // SmartDashboard.putBoolean("Controller: Left Bumper", controller.getRawButton(Constants.LEFT_BUMPER));
    // SmartDashboard.putBoolean("Controller: X Button", controller.getRawButton(Constants.X_BUTTON));
    // SmartDashboard.putBoolean("Controller: Y Button", controller.getRawButton(Constants.Y_BUTTON));
    // SmartDashboard.putBoolean("Controller: B Button", controller.getRawButton(Constants.B_BUTTON));
    // SmartDashboard.putBoolean("Controller: A Button", controller.getRawButton(Constants.A_BUTTON));
     SmartDashboard.putNumber("Controller: Left Joystick X Ais", driveController.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS));
     SmartDashboard.putNumber("Controller: Left Joystick Y Axis", driveController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
     SmartDashboard.putNumber("Controller: Right Joystick X Axis", driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
     SmartDashboard.putNumber("Controller: Right Joystick Y Axis", driveController.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS));

  if (Robot.manualDriveControl) {
    double ySpeed = driveController.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
    double xSpeed = -driveController.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS);
    double zSpeed = -driveController.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS);
    
    // Speed limits
    ySpeed = Math.max(Math.min(ySpeed, 0.4), -0.4);
    xSpeed = Math.max(Math.min(xSpeed, 0.4), -0.4);
    zSpeed = Math.max(Math.min(zSpeed, 0.4), -0.4);

    if (Math.abs(zSpeed) > 0.01) { // If we are telling the robot to rotate, then let it rotate
			// m_driveSubsystem.driveCartesian(ySpeed, xSpeed, zSpeed, m_driveSubsystem.getRotation2d()); // field-relative
      m_driveSubsystem.driveCartesian(ySpeed, xSpeed, zSpeed); // robot-relative
			goalAngle = m_driveSubsystem.getGyroAngle();
		}
		else { // Otherwise, use the gyro to maintain our current angle
			double error = m_driveSubsystem.getGyroAngle() - goalAngle;
			
			double correction = Constants.GYRO_TURN_KP * error;
      if (Math.abs(correction) > Constants.MAX_POWER_GYRO) { // Maximum value we want
        correction = Math.copySign(Constants.MAX_POWER_GYRO, correction);
      }
			
			// m_driveSubsystem.driveCartesian(ySpeed, xSpeed, -1 * correction, m_driveSubsystem.getRotation2d()); // field-relative
      m_driveSubsystem.driveCartesian(ySpeed, xSpeed, -1 * correction); // robot-relative
      }
    } else {
      goalAngle = m_driveSubsystem.getGyroAngle();
		}
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    System.out.println("TEST MODE STARTED");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
   * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    // Intake Controls //
    // new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).whileTrue(new IntakeSetBarPowerCommand(Constants.INTAKE_BAR_SPEED)); // Intake 

    // new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.HOLD_ALGAE_POSITION)); // Set arm position
    // new Trigger(() -> controller.getRawButton(Constants.B_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.HOLD_CORAL_POSITION)); // Set arm position
    // new Trigger(() -> controller.getRawButton(Constants.Y_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.PICK_UP_ALGAE_POSITION)); // Set arm position
    // new Trigger(() -> controller.getRawButton(Constants.X_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.PICK_UP_CORAL_POSITION)); // Set arm position

  }
}