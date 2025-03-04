// src/main/java/frc/robot/subsystems/VisionSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * VisionSubsystem - ROBOT EYES AND BRAINS!
 * 
 * Uses PhotonVision for ball tracking and field positioning.
 * 
 * Pipeline 0: Ball detection (color + shape filtering)
 * Pipeline 1: AprilTag detection (field positioning)
 * 
 * coded by paysean
 */
public class VisionSubsystem extends SubsystemBase {
    // ========== HARDWARE SETUP ==========
    private final PhotonCamera m_camera;
    
    // ========== VISION PROCESSING DATA ==========
    private PhotonPipelineResult m_latestResult;
    private int m_currentPipeline = 0;
    
    // ========== CAMERA MOUNTING DATA ==========
    // These values affect distance calculations!
    private final double CAMERA_HEIGHT_METERS = 0.45;  // Camera height from floor
    private final double CAMERA_PITCH_RADIANS = 0.15;  // ~8.6° downward tilt
    
    // ========== TARGET HEIGHTS ==========
    private final double BALL_HEIGHT_METERS = 0.12;   // Ball height from floor
    
    /**
     * Creates a new VisionSubsystem - THE ROBOT EYES!
     */
    public VisionSubsystem() {
        // Initialize the PhotonVision camera
        m_camera = new PhotonCamera("gloworm");  // Camera name in PhotonVision UI
        
        // Start in ball detection mode (pipeline 0)
        setPipeline(0);
        
        System.out.println("\n" +
                           "  👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️\n" +
                           "  👁️  VISION SYSTEM ONLINE  👁️\n" +
                           "  👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️👁️");
    }
    
    @Override
    public void periodic() {
        // Get the latest vision processing results
        m_latestResult = m_camera.getLatestResult();
        
        // Push data to SmartDashboard
        SmartDashboard.putBoolean("Target Detected", getHasTarget());
        
        if (getHasTarget()) {
            // We have a target! Show its data
            PhotonTrackedTarget target = getBestTarget();
            SmartDashboard.putNumber("Target Yaw", target.getYaw());
            SmartDashboard.putNumber("Target Pitch", target.getPitch());
            SmartDashboard.putNumber("Target Area", target.getArea());
            SmartDashboard.putNumber("Target Distance", getTargetDistance());
            
            // If we're in AprilTag mode, show the tag ID
            if (m_currentPipeline == 1 && target.getFiducialId() >= 0) {
                SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
            }
        }
    }
    
    /**
     * Check if we have any targets in view
     * 
     * @return true if targets detected
     */
    public boolean getHasTarget() {
        return m_latestResult != null && m_latestResult.hasTargets();
    }
    
    /**
     * Get the best target from vision system
     * 
     * @return The best target (null if none found)
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
     * @param id The AprilTag ID to look for
     * @return Target with matching ID (null if not found)
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
     * Get approximate distance to target in meters
     * 
     * @return Distance in meters (0 if no target)
     */
    public double getTargetDistance() {
        if (getHasTarget()) {
            PhotonTrackedTarget target = getBestTarget();
            
            // Get 3D position data if available
            if (target.getBestCameraToTarget() != null) {
                Transform3d transform = target.getBestCameraToTarget();
                
                // Calculate 3D distance
                return Math.sqrt(
                    Math.pow(transform.getX(), 2) +
                    Math.pow(transform.getY(), 2) +
                    Math.pow(transform.getZ(), 2)
                );
            } else {
                // Fallback to simple angle-based calculation
                double targetPitchRadians = Math.toRadians(target.getPitch());
                
                // Use camera height and target height for calculation
                double heightDifference = CAMERA_HEIGHT_METERS - BALL_HEIGHT_METERS;
                
                // Basic trigonometry: distance = height / tan(angle)
                double angle = CAMERA_PITCH_RADIANS + targetPitchRadians;
                if (angle == 0) return 0; // Avoid division by zero
                
                return heightDifference / Math.tan(angle);
            }
        }
        
        return 0; // No target = no distance
    }
    
    /**
     * Switch vision processing pipeline
     * 
     * @param pipelineIndex 0=ball detection, 1=AprilTag
     */
    public void setPipeline(int pipelineIndex) {
        m_camera.setPipelineIndex(pipelineIndex);
        m_currentPipeline = pipelineIndex;
        
        // Print which vision mode we're in
        String pipelineName = (pipelineIndex == 0) ? "BALL TRACKING" : "APRILTAG DETECTION";
        System.out.println("👁️ VISION MODE: " + pipelineName);
    }
    
    /**
     * Get current pipeline index
     * 
     * @return Current pipeline index
     */
    public int getCurrentPipeline() {
        return m_currentPipeline;
    }
    
    /**
     * Enable/disable driver mode (raw camera feed)
     * 
     * @param driverMode true for driver mode, false for vision processing
     */
    public void setDriverMode(boolean driverMode) {
        m_camera.setDriverMode(driverMode);
    }
}
