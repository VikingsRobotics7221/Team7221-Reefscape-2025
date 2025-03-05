/*
 * ================================================================
 *  _    _  ____   ____  _  __  _____  _    _ ____   _____       
 * | |  | |/ __ \ / __ \| |/ / / ____|| |  | |  _ \ / ____|      
 * | |__| | |  | | |  | | ' / | (___  | |  | | |_) | (___        
 * |  __  | |  | | |  | |  <   \___ \ | |  | |  _ < \___ \       
 * | |  | | |__| | |__| | . \  ____) || |__| | |_) |____) |      
 * |_|  |_|\____/ \____/|_|\_\|_____/  \____/|____/|_____/       
 *                                                                
 * ================================================================
 * 
 * TEAM 7221 - REEFSCAPE 2025 - HOOK SUBSYSTEM
 * 
 * "Hook it, hang it, conquer all!"
 * 
 * This is our epic linear actuator hook system for grabbing the barge
 * during endgame. It takes precision timing and control to get those
 * sweet endgame points that'll make us UNSTOPPABLE at Regionals!
 * 
 * Designed by Team 7221 - The Vikings
 * Coded by paysean - March 2025
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants;

/**
 * HookSubsystem - Our vertical barge hook system
 * 
 * This subsystem controls our linear actuator with J-hook end effector
 * for latching onto the barge in endgame. It extends upward from the robot,
 * hooks onto the barge, and locks for maximum pulling force. The system
 * includes safety features to prevent damage to our robot and the field.
 * 
 * I spent two weeks perfecting this design and the code is DIALED IN.
 */
public class HookSubsystem extends SubsystemBase {
    // ===== HARDWARE COMPONENTS =====
    private final SparkMax m_hookMotor; // Controls linear actuator extension/retraction
    private final DigitalInput m_extendedLimitSwitch; // Detects full extension
    private final DigitalInput m_retractedLimitSwitch; // Detects full retraction
    
    // ===== STATE TRACKING =====
    private boolean m_isExtended = false; // Tracks if hook is fully extended
    private boolean m_isRetracting = false; // Tracks if hook is currently retracting
    private long m_lastExtendTime = 0; // For tracking extension duration
    private double m_currentHookSpeed = 0.0; // Current motor speed
    private double m_peakCurrent = 0.0; // For tracking maximum current draw
    private int m_cycleCount = 0; // Tracks how many times we've cycled the hook
    
    /**
     * Creates a new HookSubsystem - our barge-grabbing machine!
     */
    public HookSubsystem() {
        System.out.println("");
        System.out.println("   /\\");
        System.out.println("  /  \\");
        System.out.println(" /    \\   INITIALIZING HOOK SUBSYSTEM");
        System.out.println("/______\\  PREPARE FOR BARGE DOMINATION!!!");
        System.out.println("");
        
        // Initialize the actuator motor controller
        m_hookMotor = new SparkMax(Constants.HOOK_MOTOR_ID, MotorType.kBrushless);
        
        // Configure for optimal performance
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(Constants.HOOK_MOTOR_INVERTED).idleMode(IdleMode.kBrake);
        m_hookMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Initialize limit switches for position detection
        m_extendedLimitSwitch = new DigitalInput(Constants.HOOK_EXTENDED_LIMIT_SWITCH_PORT);
        m_retractedLimitSwitch = new DigitalInput(Constants.HOOK_RETRACTED_LIMIT_SWITCH_PORT);
        
        // Default to retracted state
        m_isExtended = false;
        m_isRetracting = false;
        
        // Display initialization success message
        System.out.println("==================================================");
        System.out.println(">> HOOK SUBSYSTEM ONLINE AND READY FOR BATTLE!!! <<");
        System.out.println(">> BARGE CAPTURE CAPABILITY: MAXIMUM             <<");
        System.out.println("==================================================");
    }
    
    /**
     * Extends the hook upward - DEPLOY THE J-HOOK!
     * Uses a ramping mechanism for smooth extension
     */
    public void extendHook() {
        // Only extend if not already extended and limit switch isn't triggered
        if (!isExtended() && !m_extendedLimitSwitch.get()) {
            // Start with slow speed and ramp up for smooth movement
            long timeSinceStart = System.currentTimeMillis() - m_lastExtendTime;
            
            if (m_lastExtendTime == 0) {
                // First call to extend, initialize timing and start slow
                m_lastExtendTime = System.currentTimeMillis();
                m_currentHookSpeed = Constants.HOOK_EXTEND_MIN_SPEED;
                System.out.println(">> INITIATING HOOK EXTENSION SEQUENCE <<");
            } else if (timeSinceStart < 500) {
                // Smooth acceleration ramp during first 500ms
                double rampProgress = timeSinceStart / 500.0;
                m_currentHookSpeed = Constants.HOOK_EXTEND_MIN_SPEED + 
                    (Constants.HOOK_EXTEND_MAX_SPEED - Constants.HOOK_EXTEND_MIN_SPEED) * rampProgress;
            } else {
                // Full speed after 500ms
                m_currentHookSpeed = Constants.HOOK_EXTEND_MAX_SPEED;
            }
            
            // Apply calculated speed to motor
            m_hookMotor.set(m_currentHookSpeed);
            m_isRetracting = false;
            
            // Add occasional status updates but don't spam
            if (timeSinceStart % 1000 < 20 && timeSinceStart > 500) {
                System.out.println(">> HOOK EXTENDING: " + 
                                   (int)(m_currentHookSpeed * 100) + "% POWER <<");
            }
        } else {
            // Either fully extended or limit switch triggered, stop motor
            m_hookMotor.set(0);
            
            // Update extended status if newly extended
            if (!m_isExtended && m_extendedLimitSwitch.get()) {
                m_isExtended = true;
                m_cycleCount++;
                System.out.println(">> HOOK FULLY EXTENDED! READY TO GRAB BARGES!!!");
                System.out.println(">> CYCLE COUNT: " + m_cycleCount);
            }
            
            // Reset extension timer
            m_lastExtendTime = 0;
        }
    }
    
    /**
     * Retracts the hook - PULL IT BACK!
     * This is the most critical part after hooking onto the barge
     */
    public void retractHook() {
        // Only retract if not already retracted and limit switch isn't triggered
        if (!isRetracted() && !m_retractedLimitSwitch.get()) {
            // Set motor to retraction speed with pulling force
            double retractSpeed = -Constants.HOOK_RETRACT_SPEED;
            
            // Apply motor power
            m_hookMotor.set(retractSpeed);
            m_isRetracting = true;
            m_isExtended = false;
            
            // Monitor current during retraction - critical for detecting load
            double current = m_hookMotor.getOutputCurrent();
            if (current > m_peakCurrent) {
                m_peakCurrent = current;
                if (m_peakCurrent > Constants.HOOK_MAX_CURRENT * 0.8) {
                    System.out.println(">> HIGH LOAD DETECTED DURING RETRACTION: " + 
                                      m_peakCurrent + "A <<");
                }
            }
        } else {
            // Either fully retracted or limit switch triggered, stop motor
            m_hookMotor.set(0);
            
            // Update retraction status if newly retracted
            if (m_isRetracting && m_retractedLimitSwitch.get()) {
                m_isRetracting = false;
                System.out.println(">> HOOK FULLY RETRACTED! READY FOR NEXT DEPLOYMENT!");
                System.out.println(">> PEAK CURRENT DURING CYCLE: " + m_peakCurrent + "A");
                
                // Reset peak current for next cycle
                m_peakCurrent = 0.0;
            }
        }
        
        // Reset extension timer when retracting
        m_lastExtendTime = 0;
    }
    
    /**
     * Stops the hook motor immediately - EMERGENCY BRAKE
     */
    public void stopHook() {
        m_hookMotor.set(0);
        m_lastExtendTime = 0;
        System.out.println(">> HOOK MOTION STOPPED <<");
    }
    
    /**
     * Checks if hook is fully extended
     * 
     * @return true if at maximum extension
     */
    public boolean isExtended() {
        return m_isExtended || m_extendedLimitSwitch.get();
    }
    
    /**
     * Checks if hook is fully retracted
     * 
     * @return true if completely retracted
     */
    public boolean isRetracted() {
        return m_retractedLimitSwitch.get();
    }
    
    /**
     * Gets current hook position status as text
     * Useful for displaying on dashboard
     * 
     * @return Position as string: "EXTENDED", "RETRACTING", "RETRACTED", or "MOVING"
     */
    public String getHookPosition() {
        if (isExtended()) {
            return "EXTENDED";
        } else if (m_isRetracting) {
            return "RETRACTING";
        } else if (isRetracted()) {
            return "RETRACTED";
        } else {
            return "MOVING";
        }
    }
    
    /**
     * Calculate estimated hook height based on motor position
     * This is an approximation since we don't have an encoder
     * 
     * @return Estimated height in inches
     */
    public double getEstimatedHeight() {
        // This is just an approximation since we rely on limit switches
        if (isRetracted()) {
            return 0.0;
        } else if (isExtended()) {
            return 6.0; // 6 inch stroke length
        } else {
            // Rough estimate based on motor output
            return Math.abs(m_hookMotor.get()) * 6.0;
        }
    }
    
    @Override
    public void periodic() {
        // Update dashboard with hook status - ESSENTIAL FOR DRIVERS
        SmartDashboard.putBoolean("Hook Extended", isExtended());
        SmartDashboard.putBoolean("Hook Retracted", isRetracted());
        SmartDashboard.putString("Hook Status", getHookPosition());
        SmartDashboard.putNumber("Hook Speed", m_currentHookSpeed);
        SmartDashboard.putNumber("Hook Current", m_hookMotor.getOutputCurrent());
        SmartDashboard.putNumber("Peak Current", m_peakCurrent);
        SmartDashboard.putNumber("Cycle Count", m_cycleCount);
        SmartDashboard.putNumber("Est. Height", getEstimatedHeight());
        
        // Safety check - if current is too high, stop motor
        double current = m_hookMotor.getOutputCurrent();
        if (current > Constants.HOOK_MAX_CURRENT) {
            System.out.println(">> !!! HOOK CURRENT CRITICAL: " + current + "A !!! <<");
            System.out.println(">> !!! EMERGENCY STOP ACTIVATED !!! <<");
            stopHook();
        }
        
        // Track peak current for diagnostics
        if (current > m_peakCurrent) {
            m_peakCurrent = current;
        }
    }
    
    /**
     * Emergency stop sequence - ABORT OPERATION
     * Use this when things go wrong or for emergency match shutdown
     */
    public void emergencyStop() {
        // Immediately cut power to motor
        m_hookMotor.set(0);
        
        // Display emergency shutdown notification
        System.out.println("");
        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        System.out.println("!! EMERGENCY STOP ACTIVATED   !!");
        System.out.println("!! HOOK MECHANISM SHUTDOWN    !!");
        System.out.println("!! MECHANICAL INSPECTION NEEDED !!");
        System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        System.out.println("");
    }
    
    /**
     * Check overall hook system health
     * 
     * @return true if all systems are operational
     */
    public boolean checkSystemHealth() {
        boolean limitsWorking = m_extendedLimitSwitch.get() != m_retractedLimitSwitch.get();
        boolean motorResponding = m_hookMotor.getOutputCurrent() > 0.1;
        
        if (!limitsWorking) {
            System.out.println(">> WARNING: Limit switch failure detected!");
        }
        
        if (!motorResponding) {
            System.out.println(">> WARNING: Motor not drawing current!");
        }
        
        return limitsWorking && motorResponding;
    }
}
