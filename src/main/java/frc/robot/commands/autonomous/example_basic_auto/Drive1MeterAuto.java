// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.autonomous.example_basic_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;

/** Autonomous Mode (Default) ******************************************************
 * This basic autonomous routine drives forward 1 meter using encoder feedback */
public class Drive1MeterAuto extends SequentialCommandGroup {

  // List commands here sequentially
  public Drive1MeterAuto() { // List commands here sequentially
    addCommands(new Drivetrain_GyroStraight(1.0, 0.3));
  }
}