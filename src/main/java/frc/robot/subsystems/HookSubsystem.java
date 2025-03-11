// src/main/java/frc/robot/subsystems/HookSubsystem.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║                                                                          ║
 * ║   ██╗  ██╗ ██████╗  ██████╗ ██╗  ██╗    ███████╗██╗   ██╗███████╗      ║
 * ║   ██║  ██║██╔═══██╗██╔═══██╗██║ ██╔╝    ██╔════╝╚██╗ ██╔╝██╔════╝      ║
 * ║   ███████║██║   ██║██║   ██║█████╔╝     ███████╗ ╚████╔╝ ███████╗      ║
 * ║   ██╔══██║██║   ██║██║   ██║██╔═██╗     ╚════██║  ╚██╔╝  ╚════██║      ║
 * ║   ██║  ██║╚██████╔╝╚██████╔╝██║  ██╗    ███████║   ██║   ███████║      ║
 * ║   ╚═╝  ╚═╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═╝    ╚══════╝   ╚═╝   ╚══════╝      ║
 * ║                                                                          ║
 * ║   T E A M  7 2 2 1  -  R E E F S C A P E  B A R G E  D O M I N A T I O N  ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * The HookSubsystem controls the vertical J-hook mechanism that grabs the barge
 * during endgame. This system is crucial for manipulating field elements and
 * scoring endgame points.
 * 
 * FUNCTIONALITY:
 * - Precise linear extension control
 * - Position tracking via limit switches
 * - Current monitoring for safety
 * - Emergency stop capabilities
 * 
 * MECHANICAL INTERFACE:
 * - Controls a 6" stroke linear actuator
 * - Uses limit switches to detect fully extended/retracted positions
 * - Interfaces with a J-hook end effector for barge grabbing
 * 
 * IMPORTANT NOTE: This version uses standard PWM motor controllers. For SparkMAX
 * support, uncomment the SparkMAX sections and comment out the PWM implementation.
 * 
 * Control Architecture:
 * ┌───────────┐     ┌────────────┐     ┌────────────┐     ┌───────────┐
 * │ Driver    │────>│ Commands   │────>│ Subsystem  │────>│ Linear    │
 * │ Controls  │     │ (Hook)     │     │ Methods    │     │ Actuator  │
 * └───────────┘     └────────────┘     └────────────┘     └───────────┘
 *                                           │
 *                                           ▼
 *                                     ┌────────────┐
 *                                     │ Position   │
 *                                     │ Feedback   │
 *                                     └────────────┘
 */
public class HookSubsystem extends SubsystemBase {

    // ===== HARDWARE COMPONENTS =====
    
    /* NOTE: This implementation uses PWM motor controllers for compatibility.
     * To use CAN SparkMAX controllers, comment out this section and uncomment
     * the CANSparkMax section below once libraries are available.
     */
    private final PWMVictorSPX m_hookMotor = new PWMVictorSPX(Constants.Electrical.HOOK_MOTOR_ID);
    private final DigitalInput m_extendedLimitSwitch;
    private final DigitalInput m_retractedLimitSwitch;
    
    /*
     * ===== SPARKMAX IMPLEMENTATION (COMMENTED OUT) =====
     * Uncomment this section when REV libraries are available
     *
     * private CANSparkMax m_hookMotor;
     */
    
    // ===== STATE TRACKING =====
    private boolean m_isExtended = false;     // Tracks if hook is fully extended
    private boolean m_isRetracting = false;   // Tracks if hook is currently retracting
    private long m_lastExtendTime = 0;        // For tracking extension duration
    private double m_currentHookSpeed = 0.0;  // Current motor speed
    private double m_peakCurrent = 0.0;       // For tracking maximum current draw
    private int m_cycleCount = 0;             // Tracks how many times hook has been cycled
    private Timer m_safetyTimer = new Timer(); // For monitoring operation time
    
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
        
        // Initialize motor controller
        configureMotorController();
        
        // Initialize limit switches for position detection
        m_extendedLimitSwitch = new DigitalInput(Constants.Electrical.HOOK_EXTENDED_LIMIT_SWITCH_PORT);
        m_retractedLimitSwitch = new DigitalInput(Constants.Electrical.HOOK_RETRACTED_LIMIT_SWITCH_PORT);
        
        // Default to retracted state
        m_isExtended = false;
        m_isRetracting = false;
        
        // Start safety timer
        m_safetyTimer.start();
        
        // Display initialization success message
        System.out.println("==================================================");
        System.out.println(">> HOOK SUBSYSTEM ONLINE AND READY FOR BATTLE!!! <<");
        System.out.println(">> BARGE CAPTURE CAPABILITY: MAXIMUM             <<");
        System.out.println("==================================================");
    }
    
    /**
     * Configure the motor controller with optimal settings
     */
    private void configureMotorController() {
        // Configure PWM controller
        m_hookMotor.setInverted(Constants.Hook.MOTOR_INVERTED);
        
        /* 
         * ===== SPARKMAX CONFIGURATION (COMMENTED OUT) =====
         * This would be used if we had SparkMAX controllers
         * 
         * m_hookMotor = new CANSparkMax(Constants.Electrical.HOOK_MOTOR_ID, MotorType.kBrushless);
         * m_hookMotor.restoreFactoryDefaults();
         * m_hookMotor.setInverted(Constants.Hook.MOTOR_INVERTED);
         * m_hookMotor.setIdleMode(IdleMode.kBrake);  // Brake mode essential for holding position
         * m_hookMotor.setSmartCurrentLimit(30);      // 30A limit protects the motor
         * m_hookMotor.enableVoltageCompensation(11.0);
         * m_hookMotor.setOpenLoopRampRate(0.2);      // 200ms ramp for smooth acceleration
         * m_hookMotor.burnFlash();
         */
        
        System.out.println(">> Hook motor configured for optimal performance");
    }
    
    /**
     * Called periodically by the CommandScheduler
     */
    @Override
    public void periodic() {
        // Update dashboard with hook status - ESSENTIAL FOR DRIVERS
        SmartDashboard.putBoolean("Hook Extended", isExtended());
        SmartDashboard.putBoolean("Hook Retracted", isRetracted());
        SmartDashboard.putString("Hook Status", getHookPosition());
        SmartDashboard.putNumber("Hook Speed", m_currentHookSpeed);
        SmartDashboard.putNumber("Cycle Count", m_cycleCount);
        SmartDashboard.putNumber("Est. Height", getEstimatedHeight());
        
        // Safety check - For PWM controllers we can only check timing
        // With SparkMAX we would check current as well
        
        // If the motor has been running in the same direction for too long, it might be stalled
        if (Math.abs(m_currentHookSpeed) > 0.05) {
            if (m_safetyTimer.get() > 5.0) {
                System.out.println(">> !!! HOOK OPERATION TIMEOUT: " + m_safetyTimer.get() + "s !!! <<");
                System.out.println(">> !!! EMERGENCY STOP ACTIVATED !!! <<");
                stopHook();
                m_safetyTimer.reset();
            }
        } else {
            // Reset safety timer when motor is stopped
            m_safetyTimer.reset();
        }
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
                m_currentHookSpeed = Constants.Hook.EXTEND_MIN_SPEED;
                System.out.println(">> INITIATING HOOK EXTENSION SEQUENCE <<");
                m_safetyTimer.reset();
                m_safetyTimer.start();
            } else if (timeSinceStart < 500) {
                // Smooth acceleration ramp during first 500ms
                double rampProgress = timeSinceStart / 500.0;
                m_currentHookSpeed = Constants.Hook.EXTEND_MIN_SPEED + 
                    (Constants.Hook.EXTEND_MAX_SPEED - Constants.Hook.EXTEND_MIN_SPEED) * rampProgress;
            } else {
                // Full speed after 500ms
                m_currentHookSpeed = Constants.Hook.EXTEND_MAX_SPEED;
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
            m_currentHookSpeed = 0;
            
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
            double retractSpeed = -Constants.Hook.RETRACT_SPEED;
            
            // Apply motor power
            m_hookMotor.set(retractSpeed);
            m_currentHookSpeed = retractSpeed;
            m_isRetracting = true;
            m_isExtended = false;
            
            // Reset safety timer for new operation
            m_safetyTimer.reset();
            m_safetyTimer.start();
            
            // With SparkMAX, we would monitor current during retraction
            // In this PWM version, we can only track operation time
        } else {
            // Either fully retracted or limit switch triggered, stop motor
            m_hookMotor.set(0);
            m_currentHookSpeed = 0;
            
            // Update retraction status if newly retracted
            if (m_isRetracting && m_retractedLimitSwitch.get()) {
                m_isRetracting = false;
                System.out.println(">> HOOK FULLY RETRACTED! READY FOR NEXT DEPLOYMENT!");
                
                // With SparkMAX we would report peak current
                System.out.println(">> SUCCESSFUL RETRACTION CYCLE COMPLETED");
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
        m_currentHookSpeed = 0;
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
     * Calculate estimated hook height based on time
     * This is an approximation since we don't have an encoder
     * 
     * @return Estimated height in inches
     */
    public double getEstimatedHeight() {
        // PWM controllers don't have position feedback, so we estimate
        // With SparkMAX, we could read the encoder position directly
        
        // This is just an approximation
        if (isRetracted()) {
            return 0.0;
        } else if (isExtended()) {
            return 6.0; // 6 inch stroke length
        } else {
            // Rough estimate based on motor output and time
            // This is very approximate without encoder feedback
            return Math.abs(m_currentHookSpeed) * 6.0;
        }
    }
    
    /**
     * Emergency stop sequence - ABORT OPERATION
     * Use this when things go wrong or for emergency match shutdown
     */
    public void emergencyStop() {
        // Immediately cut power to motor
        m_hookMotor.set(0);
        m_currentHookSpeed = 0;
        
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
        
        // With SparkMAX we would check motor response too
        
        if (!limitsWorking) {
            System.out.println(">> WARNING: Limit switch failure detected!");
        }
        
        return limitsWorking;
    }
    
    /**
     * Applies a gentle jog to the hook to free it if it's stuck
     * Useful for clearing minor mechanical obstructions
     * 
     * @param direction Direction to jog (true = up, false = down)
     * @param duration Time in seconds to apply the jog
     */
    public void jogHook(boolean direction, double duration) {
        // Safety check - don't jog against limit switches
        if ((direction && isExtended()) || (!direction && isRetracted())) {
            System.out.println(">> Cannot jog - already at limit!");
            return;
        }
        
        // Apply a gentle motor power in the requested direction
        double jogPower = direction ? 0.3 : -0.3;
        m_hookMotor.set(jogPower);
        
        // PWM motor controllers don't have precise timing control
        // In a real implementation, we would use a Command with timeout
        // This simple implementation is just for the subsystem API
        
        System.out.println(">> Jogging hook " + (direction ? "UP" : "DOWN") + 
                           " for " + duration + " seconds");
        
        // Note: In practice, the jogHook method should be called from a Command
        // that handles the timing and motor stoppage
    }
    
    /**
     * Provides direct control of the hook motor
     * Use with caution! This bypasses safety limits.
     * Should only be used in special cases when limit switches fail.
     * 
     * @param power Power level (-1.0 to 1.0)
     */
    public void setDirectHookPower(double power) {
        // Safety bounds check
        power = Math.max(-1.0, Math.min(1.0, power));
        
        // Log warning since this bypasses normal safety
        System.out.println(">> WARNING: Direct hook control at " + power + " power");
        
        // Set motor power directly
        m_hookMotor.set(power);
        m_currentHookSpeed = power;
        
        // Reset safety timer
        m_safetyTimer.reset();
        m_safetyTimer.start();
    }
}
