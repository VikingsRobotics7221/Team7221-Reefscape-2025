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
 * Drivetrain_SmoothPath - ADVANCED PATH FOLLOWING WITH 16:1 OPTIMIZATION!
 * 
 * This command generates and follows smooth paths optimized for our 16:1 drivetrain.
 * It uses WPILib's trajectory generation but with CUSTOM PID TUNING specifically
 * for high-torque, high-precision movement with our tank drive.
 * 
 * Unlike basic autonomous commands, this one:
 * 1. Creates curved paths between waypoints
 * 2. Dynamically adjusts speed based on path curvature
 * 3. Uses encoder feedback for closed-loop position tracking
 * 4. Handles smooth acceleration/deceleration automatically
 * 
 * coded by paysean - this is my MASTERPIECE of motion control!
 */
public class Drivetrain_SmoothPath extends Command {
    
    // The drivetrain subsystem to control
    private final DriveSubsystem m_drive;
    
    // Path generation and following objects
    private final Trajectory m_trajectory;
    private final RamseteController m_ramseteController = new RamseteController();
    private final PIDController m_leftController = new PIDController(Constants.kP_FRONT_LEFT_VELOCITY, 0, 0);
    private final PIDController m_rightController = new PIDController(Constants.kP_FRONT_RIGHT_VELOCITY, 0, 0);
    
    // Timing
    private final Timer m_timer = new Timer();
    private final double m_timeoutSeconds;
    
    // Tracking variables
    private boolean m_isFinished = false;
    private double m_previousTime = 0;
    private double m_maxPathError = 0;
    
    // ASCII ART because it makes code better!
    //    _____                 _   _     
    //   / ____|               | | | |    
    //  | (___  _ __ ___   ___ | |_| |__  
    //   \___ \| '_ ` _ \ / _ \| __| '_ \ 
    //   ____) | | | | | | (_) | |_| | | |
    //  |_____/|_| |_| |_|\___/ \__|_| |_|
    //   _____      _   _                 
    //  |  __ \    | | | |                
    //  | |__) |_ _| |_| |__              
    //  |  ___/ _` | __| '_ \             
    //  | |  | (_| | |_| | | |            
    //  |_|   \__,_|\__|_| |_|            
    
    /**
     * Creates a smooth path command between waypoints
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
        
        // Add this subsystem as a requirement
        addRequirements(m_drive);
        
        // Configure trajectory constraints - SPECIALLY TUNED FOR 16:1 RATIO!
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration)
            .setKinematics(m_drive.getKinematics())
            .setReversed(reversed)
            .addConstraint(new CentripetalAccelerationConstraint(Constants.kMAX_ANGULAR_SPEED_RADIANS_PER_SECOND));
        
        // Generate the trajectory
        m_trajectory = TrajectoryGenerator.generateTrajectory(
            startPose,
            waypoints,
            endPose,
            config
        );
        
        System.out.println("");
        System.out.println(">> SMOOTH PATH COMMAND CREATED!");
        System.out.println(">> TOTAL PATH LENGTH: " + m_trajectory.getTotalTimeSeconds() + " seconds");
        System.out.println(">> WAYPOINTS: " + waypoints.size());
        System.out.println("");
    }
    
    /**
     * Simplified constructor for common path following
     * 
     * @param waypoints Array of x,y points to visit [x1,y1,x2,y2,...]
     * @param endHeading Final heading in degrees
     * @param maxVelocity Maximum velocity in m/s
     * @param timeoutSeconds Maximum execution time
     */
    public Drivetrain_SmoothPath(double[] waypoints, double endHeading, double maxVelocity, double timeoutSeconds) {
        this(
            new Pose2d(0, 0, new Rotation2d(0)),  // Start at current position
            waypointsArrayToList(waypoints),      // Convert array to Translation2d List
            new Pose2d(
                waypoints[waypoints.length-2],    // Last x
                waypoints[waypoints.length-1],    // Last y
                Rotation2d.fromDegrees(endHeading) // End heading
            ),
            maxVelocity,
            maxVelocity * 0.75,  // 75% of max velocity for acceleration limit
            timeoutSeconds,
            false  // Forward
        );
    }
    
    /**
     * Helper method to convert array of coordinates to List of Translation2d
     */
    private static List<Translation2d> waypointsArrayToList(double[] waypoints) {
        List<Translation2d> waypointsList = new ArrayList<>();
        
        // Skip first and last points (they're start/end poses)
        for (int i = 0; i < waypoints.length - 3; i += 2) {
            waypointsList.add(new Translation2d(waypoints[i], waypoints[i+1]));
        }
        
        return waypointsList;
    }

    @Override
    public void initialize() {
        // Reset the timer
        m_timer.reset();
        m_timer.start();
        m_previousTime = 0;
        m_maxPathError = 0;
        
        // Reset odometry to the starting pose
        m_drive.resetOdometry(m_trajectory.getInitialPose());
        
        System.out.println(">> STARTING SMOOTH PATH FOLLOW");
        System.out.println(">> INITIAL POSE: " + m_trajectory.getInitialPose());
        System.out.println(">> FINAL POSE: " + m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters);
    }

    @Override
    public void execute() {
        // Get the time elapsed
        double currentTime = m_timer.get();
        
        // Get the desired state at the current time
        Trajectory.State desiredState = m_trajectory.sample(currentTime);
        
        // Calculate wheel speeds using Ramsete controller
        var targetWheelSpeeds = m_drive.getKinematics().toWheelSpeeds(
            m_ramseteController.calculate(m_drive.getPose(), desiredState)
        );
        
        // Get current pose for error calculation
        Pose2d currentPose = m_drive.getPose();
        Pose2d desiredPose = desiredState.poseMeters;
        
        // Calculate path error for diagnostics
        double pathError = currentPose.getTranslation().getDistance(desiredPose.getTranslation());
        if (pathError > m_maxPathError) {
            m_maxPathError = pathError;
        }
        
        // Set the wheel speeds on the drivetrain
        m_drive.setWheelSpeeds(targetWheelSpeeds);
        
        // Only log occasionally to reduce spam
        if (currentTime - m_previousTime > 0.5) {
            System.out.printf(">> PATH: t=%.2fs, error=%.2fm, x=%.2f, y=%.2f, heading=%.1f°\n",
                            currentTime,
                            pathError,
                            currentPose.getX(),
                            currentPose.getY(),
                            currentPose.getRotation().getDegrees());
            m_previousTime = currentTime;
        }
        
        // Check if we're at the end of the trajectory
        if (currentTime >= m_trajectory.getTotalTimeSeconds()) {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Check if we've reached the end of the trajectory or timed out
        return m_isFinished || m_timer.hasElapsed(m_timeoutSeconds);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the timer
        m_timer.stop();
        
        // Stop the robot
        m_drive.arcadeDrive(0, 0);
        
        // Print completion message
        if (interrupted) {
            System.out.println(">> SMOOTH PATH INTERRUPTED after " + m_timer.get() + " seconds");
        } else {
            System.out.println(">> SMOOTH PATH COMPLETED in " + m_timer.get() + " seconds");
            System.out.println(">> MAX PATH ERROR: " + m_maxPathError + " meters");
            System.out.println(">> FINAL POSE: " + m_drive.getPose());
        }
    }
}
