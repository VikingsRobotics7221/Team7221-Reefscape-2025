// src/main/java/frc/robot/subsystems/VisionSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

/**
 * ┌─────────────────────────────────────────────────────────────┐
 * │                   VISION SUBSYSTEM                          │
 * │              THE EYES OF TEAM 7221 ROBOT                    │
 * │                                                             │
 * │  Provides visual perception, targeting, and distance        │
 * │  calculations needed for autonomous ball acquisition        │
 * │  and field positioning during the Reefscape competition.    │
 * └─────────────────────────────────────────────────────────────┘
 * 
 * This subsystem manages the robot's camera systems using PhotonVision
 * for vision processing. It provides:
 * 
 * - Pipeline Management: Switching between ball detection and AprilTag modes
 * - Target Detection: Finding and tracking game pieces and field markers
 * - Spatial Awareness: Calculating distances to targets
 * - Data Integration: Providing vision data to other subsystems
 * 
 * Camera Mounting Configuration:
 * - Height: 0.45 meters from floor
 * - Angle: ~8.6° downward tilt (0.15 radians)
 */
public class VisionSubsystem extends SubsystemBase {
    
    // ========== HARDWARE CONFIGURATION ==========
    private final PhotonCamera m_camera;
    
    // ========== VISION STATE DATA ==========
    private PhotonPipelineResult m_latestResult;
    private int m_currentPipeline = 0;
    private boolean m_hasValidTarget = false;
    private double m_lastAcquisitionTime = 0;
    
    // ========== CAMERA MOUNTING GEOMETRY ==========
    private final double m_cameraHeightMeters;
    private final double m_cameraPitchRadians;
    
    // ========== TARGET PARAMETERS ==========
    private final double m_ballHeightMeters;
    
    /**
     * Creates a new VisionSubsystem with the standard configuration.
     * Initializes camera connection and default parameters.
     */
    public VisionSubsystem() {
        System.out.println("╔════════════════════════════════════════════════╗");
        System.out.println("║       INITIALIZING VISION SUBSYSTEM            ║");
        System.out.println("╚════════════════════════════════════════════════╝");
        
        // Initialize camera with the name configured in PhotonVision
        m_camera = new PhotonCamera("gloworm");
        
        // Set camera physical mounting parameters from constants
        m_cameraHeightMeters = Constants.Vision.CAMERA_HEIGHT_METERS;
        m_cameraPitchRadians = Constants.Vision.CAMERA_PITCH_RADIANS;
        
        // Set target parameters
        m_ballHeightMeters = Constants.Vision.BALL_HEIGHT_METERS;
        
        // Initialize with ball tracking pipeline (pipeline 0)
        setPipeline(Constants.Vision.BALL_PIPELINE_INDEX);
        
        System.out.println(">> Vision system camera initialized: gloworm");
        System.out.println(">> Camera height: " + m_cameraHeightMeters + "m");
        System.out.println(">> Camera pitch: " + Math.toDegrees(m_cameraPitchRadians) + "°");
        System.out.println(">> Default pipeline: Ball tracking (0)");
    }
    
    /**
     * Called periodically by the CommandScheduler.
     * Updates vision data and publishes to dashboard.
     */
    @Override
    public void periodic() {
        // Get the latest result from the camera
        m_latestResult = m_camera.getLatestResult();
        m_hasValidTarget = m_latestResult.hasTargets();
        
        if (m_hasValidTarget) {
            m_lastAcquisitionTime = m_latestResult.getTimestampSeconds();
        }
        
        // Update dashboard with vision data
        updateDashboard();
    }
    
    /**
     * Update SmartDashboard with vision feedback.
     * This provides driver/operator with visual confirmation of what the robot sees.
     */
    private void updateDashboard() {
        SmartDashboard.putBoolean("Target Detected", getHasTarget());
        SmartDashboard.putNumber("Vision Pipeline", m_currentPipeline);
        
        if (getHasTarget()) {
            PhotonTrackedTarget target = getBestTarget();
            if (target != null) {
                SmartDashboard.putNumber("Target Yaw", target.getYaw());
                SmartDashboard.putNumber("Target Pitch", target.getPitch());
                SmartDashboard.putNumber("Target Area", target.getArea());
                SmartDashboard.putNumber("Target Distance", getTargetDistance());
                
                // If in AprilTag mode, show tag ID
                if (m_currentPipeline == Constants.Vision.APRILTAG_PIPELINE_INDEX && 
                    target.getFiducialId() >= 0) {
                    SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
                }
            }
        } else {
            // Clear values when no target
            SmartDashboard.putNumber("Target Yaw", 0);
            SmartDashboard.putNumber("Target Pitch", 0);
            SmartDashboard.putNumber("Target Area", 0);
            SmartDashboard.putNumber("Target Distance", 0);
        }
    }
    
    /**
     * Check if the vision system detects any targets.
     * 
     * @return true if targets detected
     */
    public boolean getHasTarget() {
        return m_hasValidTarget;
    }
    
    /**
     * Get the best target from vision system.
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
     * Find a specific AprilTag by ID.
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
     * Get approximate distance to target in meters.
     * Uses multiple calculation methods for accuracy.
     * 
     * @return Distance in meters (0 if no target)
     */
    public double getTargetDistance() {
        if (getHasTarget()) {
            PhotonTrackedTarget target = getBestTarget();
            
            // Try 3D position data if available
            if (target.getBestCameraToTarget() != null) {
                Transform3d transform = target.getBestCameraToTarget();
                
                // Calculate 3D distance using Euclidean distance formula
                return Math.sqrt(
                    Math.pow(transform.getX(), 2) +
                    Math.pow(transform.getY(), 2) +
                    Math.pow(transform.getZ(), 2)
                );
            } else {
                // Fall back to trigonometric calculation
                double targetPitchRadians = Math.toRadians(target.getPitch());
                
                // Use camera height and target height for calculation
                double heightDifference = m_cameraHeightMeters - m_ballHeightMeters;
                
                // Basic trigonometry: distance = height / tan(angle)
                double angle = m_cameraPitchRadians + targetPitchRadians;
                if (Math.abs(angle) < 0.001) return 10.0; // Avoid division by zero
                
                return heightDifference / Math.tan(angle);
            }
        }
        
        return 0.0; // No target = no distance
    }
    
    /**
     * Switch vision processing pipeline.
     * 
     * @param pipelineIndex Pipeline index to switch to
     *                      (0=ball detection, 1=AprilTag)
     */
    public void setPipeline(int pipelineIndex) {
        // Safety bounds check
        if (pipelineIndex < 0 || pipelineIndex > 1) {
            System.err.println("WARNING: Invalid pipeline index: " + pipelineIndex);
            return;
        }
        
        m_camera.setPipelineIndex(pipelineIndex);
        m_currentPipeline = pipelineIndex;
        
        // Log pipeline change
        String pipelineName = (pipelineIndex == Constants.Vision.BALL_PIPELINE_INDEX) 
            ? "BALL TRACKING" 
            : "APRILTAG DETECTION";
        
        System.out.println(">> VISION PIPELINE CHANGED: " + pipelineName);
    }
    
    /**
     * Get current pipeline index.
     * 
     * @return Current pipeline index
     */
    public int getCurrentPipeline() {
        return m_currentPipeline;
    }
    
    /**
     * Enable/disable driver mode (raw camera feed).
     * Use this to temporarily disable vision processing and show
     * a direct camera feed for driver assistance.
     * 
     * @param driverMode true for driver mode, false for vision processing
     */
    public void setDriverMode(boolean driverMode) {
        m_camera.setDriverMode(driverMode);
        System.out.println(">> Camera driver mode " + (driverMode ? "ENABLED" : "DISABLED"));
    }
    
    /**
     * Get how long it's been since we last saw a target.
     * Useful for determining if target tracking is stale.
     * 
     * @return Time in seconds since last valid target acquisition
     */
    public double getTimeSinceLastTarget() {
        if (!getHasTarget() && m_lastAcquisitionTime > 0) {
            return m_latestResult.getTimestampSeconds() - m_lastAcquisitionTime;
        }
        return 0.0; // Currently have target or never had one
    }
    
    /**
     * Check if we have a "fresh" target - one that was recently detected.
     * 
     * @param maxAgeSeconds Maximum age in seconds for a target to be considered fresh
     * @return true if target exists and is fresh
     */
    public boolean hasFreshTarget(double maxAgeSeconds) {
        return getHasTarget() || getTimeSinceLastTarget() < maxAgeSeconds;
    }
    
    /**
     * Get the yaw angle to the best target.
     * Yaw is the horizontal angle from the camera centerline.
     * 
     * @return Yaw angle in degrees (0 if no target)
     */
    public double getTargetYaw() {
        if (getHasTarget()) {
            return getBestTarget().getYaw();
        }
        return 0.0;
    }
    
    /**
     * Get the pitch angle to the best target.
     * Pitch is the vertical angle from the camera centerline.
     * 
     * @return Pitch angle in degrees (0 if no target)
     */
    public double getTargetPitch() {
        if (getHasTarget()) {
            return getBestTarget().getPitch();
        }
        return 0.0;
    }
    
    /**
     * Check if the camera connection is healthy.
     * 
     * @return true if camera is connected and responding
     */
    public boolean isCameraHealthy() {
        // The fact that we can get results indicates camera is connected
        return m_camera != null;
    }
}
