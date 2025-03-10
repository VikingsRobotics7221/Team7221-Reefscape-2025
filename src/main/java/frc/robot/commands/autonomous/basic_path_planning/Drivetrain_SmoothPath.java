// src/main/java/frc/robot/commands/autonomous/basic_path_planning/Drivetrain_SmoothPath.java
package frc.robot.commands.autonomous.basic_path_planning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;
import java.util.ArrayList;

/**
 * Drivetrain_SmoothPath - Advanced Path Following with Tank Drive
 * 
 * This command generates and follows smooth paths optimized for tank drive systems.
 * It creates curved trajectories between waypoints with proper acceleration and
 * deceleration profiles for reliable autonomous movement.
 * 
 * SYSTEM INTEGRATION:
 * - Uses DriveSubsystem from Robot.java for motor control
 * - References Constants.java for robot-specific parameters
 * - Utilizes WPILib trajectory classes for path generation
 * - Provides detailed feedback during operation
 * 
 * KEY FEATURES:
 * - Creates curved paths between waypoints
 * - Dynamically adjusts speed based on path curvature
 * - Uses encoder feedback for closed-loop position tracking
 * - Manages smooth acceleration/deceleration automatically
 */
public class Drivetrain_SmoothPath extends Command {
    
    // The drivetrain subsystem that will execute the path
    private final DriveSubsystem m_drive;
    
    // Path generation and following components
    private final Trajectory m_trajectory;
    private final RamseteController m_ramseteController = new RamseteController();
    private final PIDController m_leftController = new PIDController(Constants.kP_DRIVE_VEL, 0, 0);
    private final PIDController m_rightController = new PIDController(Constants.kP_DRIVE_VEL, 0, 0);
    
    // Timing and tracking variables
    private final Timer m_timer = new Timer();
    private final double m_timeoutSeconds;
    private boolean m_isFinished = false;
    private double m_previousTime = 0;
    private double m_maxPathError = 0;
    
    /**
     * Creates a path-following command between multiple waypoints.
     * 
     * @param startPose Starting position and heading
     * @param waypoints List of points to pass through
     * @param endPose Ending position and heading
     * @param maxVelocity Maximum velocity in m/s
     * @param maxAcceleration Maximum acceleration in m/s²
     * @param timeoutSeconds Maximum time to follow path
     * @param reversed Whether to drive backwards along the path
     */
    public Drivetrain_SmoothPath(
            Pose2d startPose,
            List<Translation2d> waypoints,
            Pose2d endPose,
            double maxVelocity,
            double maxAcceleration,
            double timeoutSeconds,
            boolean reversed) {
        
        m_drive = Robot.m_driveSubsystem;
        m_timeoutSeconds = timeoutSeconds;
        
        // Register that this command requires the drivetrain
        addRequirements(m_drive);
        
        // Configure trajectory constraints based on robot capabilities
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
            .setKinematics(m_drive.getKinematics())
            .setReversed(reversed)
            .addConstraint(new CentripetalAccelerationConstraint(Constants.MAX_TURN_RATE_RAD_PER_SEC));
        
        // Generate the trajectory using WPILib trajectory generator
        m_trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            waypoints,
            endPose,
            config
        );
        
        // Log path creation for debugging
        System.out.println("SmoothPath created with " + waypoints.size() + 
                           " waypoints, " + m_trajectory.getTotalTimeSeconds() + " seconds duration");
    }
    
    /**
     * Simplified constructor for common path following scenarios.
     * 
     * @param waypoints Array of x,y coordinates [x1,y1,x2,y2,...]
     * @param endHeading Final heading in degrees
     * @param maxVelocity Maximum velocity in m/s
     * @param timeoutSeconds Maximum execution time
     */
    public Drivetrain_SmoothPath(double[] waypoints, double endHeading, double maxVelocity, double timeoutSeconds) {
        this(
            new Pose2d(0, 0, new Rotation2d(0)),  // Start at current position
            waypointsArrayToList(waypoints),      // Convert array to Translation2d List
            new Pose2d(
                waypoints[waypoints.length-2],    // Last x coordinate
                waypoints[waypoints.length-1],    // Last y coordinate
                Rotation2d.fromDegrees(endHeading) // Final orientation
            ),
            maxVelocity,
            maxVelocity * 0.75,  // Limit acceleration to 75% of max velocity
            timeoutSeconds,
            false  // Forward by default
        );
    }
    
    /**
     * Helper method to convert array of coordinates to List of Translation2d.
     * This allows for simpler parameter passing when creating paths.
     */
    private static List<Translation2d> waypointsArrayToList(double[] waypoints) {
        List<Translation2d> waypointsList = new ArrayList<>();
        
        // Convert coordinate pairs to Translation2d objects
        // Skip first and last points (they're specified in start/end poses)
        for (int i = 0; i < waypoints.length - 3; i += 2) {
            waypointsList.add(new Translation2d(waypoints[i], waypoints[i+1]));
        }
        
        return waypointsList;
    }

    @Override
    public void initialize() {
        // Reset the timer and tracking variables
        m_timer.reset();
        m_timer.start();
        m_previousTime = 0;
        m_maxPathError = 0;
        
        // Reset odometry to match the starting pose of the trajectory
        m_drive.resetOdometry(m_trajectory.getInitialPose());
        
        System.out.println("Starting path following from " + m_trajectory.getInitialPose() + 
                           " to " + m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters);
    }

    @Override
    public void execute() {
        // Get the current elapsed time
        double currentTime = m_timer.get();
        
        // Get the desired robot state at the current time
        Trajectory.State desiredState = m_trajectory.sample(currentTime);
        
        // Calculate wheel speeds using Ramsete controller for path following
        var targetWheelSpeeds = m_drive.getKinematics().toWheelSpeeds(
            m_ramseteController.calculate(m_drive.getPose(), desiredState)
        );
        
        // Get current pose for error calculation
        Pose2d currentPose = m_drive.getPose();
        Pose2d desiredPose = desiredState.poseMeters;
        
        // Calculate path error (distance from desired position)
        double pathError = currentPose.getTranslation().getDistance(desiredPose.getTranslation());
        if (pathError > m_maxPathError) {
            m_maxPathError = pathError;
        }
        
        // Apply the calculated wheel speeds to the drivetrain
        m_drive.setWheelSpeeds(targetWheelSpeeds);
        
        // Periodically log progress (not every cycle to reduce console spam)
        if (currentTime - m_previousTime > 0.5) {
            System.out.printf("Path: t=%.1fs, error=%.2fm, pos=(%.2f,%.2f), heading=%.1f°\n",
                            currentTime,
                            pathError,
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getRotation().getDegrees());
            m_previousTime = currentTime;
        }
        
        // Check if we've reached the end of the trajectory
        if (currentTime >= m_trajectory.getTotalTimeSeconds()) {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Command is finished when we reach the end of the trajectory or timeout
        return m_isFinished || m_timer.hasElapsed(m_timeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the timer
        m_timer.stop();
        
        // Stop the robot
        m_drive.arcadeDrive(0, 0);
        
        // Report completion status
        if (interrupted) {
            System.out.println("Path following interrupted after " + m_timer.get() + " seconds");
        } else {
            System.out.println("Path following completed in " + m_timer.get() + " seconds");
            System.out.println("Maximum path error: " + m_maxPathError + " meters");
            System.out.println("Final position: " + m_drive.getPose());
        }
    }
}
