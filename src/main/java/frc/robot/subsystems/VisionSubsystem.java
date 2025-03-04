// src/main/java/frc/robot/subsystems/VisionSubsystem.java
package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * VisionSubsystem - THE ROBOT'S EYES!!!
 * 
 * ====================================
 *      👁️  PHOTONVISION  👁️
 * ====================================
 * 
 * This system uses the PhotonVision library to detect:
 * - AprilTags (for field positioning)
 * - Game Pieces (balls using color detection)
 * 
 * It's like giving our robot SUPER VISION POWERS!!!
 */
public class VisionSubsystem extends SubsystemBase {
    // Camera for vision tracking
    private final PhotonCamera m_camera;
    
    // Results from vision processing
    private PhotonPipelineResult m_latestResult;
    
    // Camera mounting parameters
    private final double CAMERA_HEIGHT_METERS = 0.5;  // Height of camera from floor
    private final double CAMERA_PITCH_RADIANS = 0.2;  // Slight downward angle
    
    // Game piece tracking
    private final double BALL_HEIGHT_METERS = 0.12;   // Height of ball center from floor
    
    /**
     * Creates the VisionSubsystem - THE ROBOT'S EYES!
     */
    public VisionSubsystem() {
        // Initialize the PhotonCamera with the correct camera name
        m_camera = new PhotonCamera("USB_Camera");
        
        // Set initial pipeline to ball detection (pipeline 0)
        m_camera.setPipelineIndex(0);
        
        System.out.println("");
        System.out.println("  👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️");
        System.out.println("  👁️ VISION SYSTEM ONLINE       👁️");
        System.out.println("  👁️ PHOTONVISION INITIALIZED  👁️");
        System.out.println("  👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️");
        System.out.println("");
    }
    
    @Override
    public void periodic() {
        // Get the latest results from PhotonVision
        m_latestResult = m_camera.getLatestResult();
        
        // Update SmartDashboard with vision data
        SmartDashboard.putBoolean("Has Target", getHasTarget());
        
        if (getHasTarget()) {
            PhotonTrackedTarget target = getBestTarget();
            SmartDashboard.putNumber("Target Yaw", target.getYaw());
            SmartDashboard.putNumber("Target Pitch", target.getPitch());
            SmartDashboard.putNumber("Target Area", target.getArea());
            SmartDashboard.putNumber("Target Distance", getTargetDistance());
        }
    }
    
    /**
     * Check if we have any targets in view
     * 
     * @return true if targets are detected
     */
    public boolean getHasTarget() {
        return m_latestResult != null && m_latestResult.hasTargets();
    }
    
    /**
     * Get the best target from vision system
     * 
     * @return The best target, or null if no targets
     */
    public PhotonTrackedTarget getBestTarget() {
        if (getHasTarget()) {
            return m_latestResult.getBestTarget();
        }
        return null;
    }
    
    /**
     * Find a specific AprilTag by ID
     * 
     * @param id The AprilTag ID to find
     * @return The target matching the ID, or null if not found
     */
    public PhotonTrackedTarget getTargetWithID(int id) {
        if (getHasTarget()) {
            for (PhotonTrackedTarget target : m_latestResult.getTargets()) {
                if (target.getFiducialId() == id) {
                    return target;
                }
            }
        }
        return null;
    }
    
    /**
     * Calculate the distance to the target using 3D pose estimation
     * 
     * @return Distance in meters, or 0 if no target
     */
    public double getTargetDistance() {
        if (getHasTarget()) {
            PhotonTrackedTarget target = getBestTarget();
            Transform3d targetPose = target.getBestCameraToTarget();
            
            // 3D Cartesian distance calculation
            return Math.sqrt(
                Math.pow(targetPose.getX(), 2) +
                Math.pow(targetPose.getY(), 2) +
                Math.pow(targetPose.getZ(), 2)
            );
        }
        return 0;
    }
    
    /**
     * Switch to the appropriate vision pipeline
     * 
     * @param pipelineIndex Pipeline to use (0 = ball detection, 1 = AprilTag)
     */
    public void setPipeline(int pipelineIndex) {
        m_camera.setPipelineIndex(pipelineIndex);
        
        String pipelineName = (pipelineIndex == 0) ? "BALL DETECTION" : "APRILTAG TRACKING";
        System.out.println("👁️ SWITCHING PIPELINE TO: " + pipelineName);
    }
    
    /**
     * Enable or disable the camera driver mode (raw feed instead of processed)
     * 
     * @param driverMode true for driver mode, false for vision processing
     */
    public void setDriverMode(boolean driverMode) {
        m_camera.setDriverMode(driverMode);
    }
    
    /**
     * Get the current pipeline index
     * 
     * @return Current pipeline index
     */
    public int getCurrentPipeline() {
        return m_camera.getPipelineIndex();
    }
}
