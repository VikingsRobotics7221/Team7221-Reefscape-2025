// src/main/java/frc/robot/Main.java
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * DO NOT MODIFY THIS FILE UNLESS YOU KNOW WHAT YOU'RE DOING!!!
 * 
 * This is the MAIN LAUNCHER for our EPIC Viking robot code! It's like
 * the ignition switch that starts our robot's brain! This class is the
 * very first thing that runs when our code deploys to the roboRIO.
 * 
 * SERIOUSLY - we spent HOURS debugging weird issues last year because
 * someone modified this file. DON'T TOUCH IT! Just let it do its magic!
 * 
 * Main → Robot → Victory!
 * 
 * - Team 7221 "The Vikings" Reefscape 2025
 * coded by paysean
 */
public final class Main {
  /**
   * Private constructor because this is a utility class and
   * nobody should EVER create an instance of Main!
   */
  private Main() {
    // This constructor will never run - it just prevents instantiation
    // Java programming best practice for utility classes like this one!
  }

  /**
   * THE STARTING POINT OF OUR ENTIRE ROBOT CODE!!! 
   * 
   * This is literally the first method that runs when our robot powers up.
   * It tells the WPILib framework to instantiate our Robot class and start
   * running all the initialization sequences.
   * 
   * If you change your main robot class, change the parameter type.
   * But seriously, don't mess with this unless you ABSOLUTELY need to!
   * 
   * @param args Command-line arguments (unused, but required by Java)
   */
  public static void main(String... args) {
    // BOOM! This line launches our entire robot code!
    // It creates a new Robot object and hands control to WPILib's state machine
    RobotBase.startRobot(Robot::new);
    
    // NOTE: Code after this line will NEVER execute!
    // Once startRobot runs, WPILib takes full control of the thread
  }
}
