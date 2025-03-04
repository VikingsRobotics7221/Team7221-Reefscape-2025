/*
 * ================================================================
 *  _    _  ____   ____  _  __  _____  _    _ ____   _____       !
 * | |  | |/ __ \ / __ \| |/ / / ____|| |  | |  _ \ / ____|      !
 * | |__| | |  | | |  | | ' / | (___  | |  | | |_) | (___        !
 * |  __  | |  | | |  | |  <   \___ \ | |  | |  _ < \___ \       !
 * | |  | | |__| | |__| | . \  ____) || |__| | |_) |____) |      !
 * |_|  |_|\____/ \____/|_|\_\|_____/  \____/|____/|_____/       !
 *                                                                !
 * ================================================================
 * 
 * TEAM 7221 - REEFSCAPE 2025 - HOOK SUBSYSTEM
 * 
 * "Hook it, hang it, conquer all!"
 * 
 * Totally epic barge hooking system for massive points at endgame!
 * 
 *        _____
 *       |  _  |    
 *       | |_| |    
 *       |_____|                     ,--.
 *       |  |            hook        |  | <-- barge
 *       |  |        +---------+    /    \
 *       |  |        |         |---/      \    
 *   ____|__|________|_________|__/_______\____
 *  |                                          |
 *  |            OUR EPIC ROBOT                |
 *  |__________________________________________|
 *   
 * Coded with <3 by Team 7221 - 2025
 * 
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants;

public class HookSubsystem extends SubsystemBase {
    //------------------------------------------
    // THE EPIC HOOK MOTOR & SENSORS
    //------------------------------------------
    
    /*
     *    MOTOR CONTROLLER       LIMIT SWITCHES
     *    .------------.         .----.  .----.
     *    |  SparkMAX  |         |SW1 |  |SW2 |
     *    |  for the   |         '----'  '----'
     *    |   LINEAR   |           |       |
     *    |  ACTUATOR  |           |       |
     *    '------------'           |       |
     *          |                  |       |
     *          V                  V       V
     *    EXTENDS/RETRACTS     FULLY     FULLY
     *      THE HOOK         EXTENDED   RETRACTED
     *         
     */
    
    // Motor controller for the linear actuator
    private final SparkMax m_hookMotor;
    
    // Limit switches to detect the end positions
    private final DigitalInput m_extendedLimitSwitch;
    private final DigitalInput m_retractedLimitSwitch;
    
    // State tracking
    private boolean m_isExtended = false;
    private boolean m_isRetracting = false;
    private long m_lastExtendTime = 0;
    private double m_currentHookSpeed = 0.0;
    
    /**
     * Creates a new HookSubsystem - THE BARGE DESTROYER!!!
     */
    public HookSubsystem() {
        System.out.println("");
        System.out.println("   /\\");
        System.out.println("  /  \\");
        System.out.println(" /    \\   INITIALIZING HOOK SUBSYSTEM");
        System.out.println("/______\\  PREPARE FOR BARGE DOMINATION!!!");
        System.out.println("");
        
        // Initialize motor controller for actuator
        m_hookMotor = new SparkMax(Constants.HOOK_MOTOR_ID, MotorType.kBrushed);
        
        // Configure motor settings
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(Constants.HOOK_MOTOR_INVERTED).idleMode(IdleMode.kBrake);
        m_hookMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
       // Set current limit to protect our precious actuator
// FIXED: Using setCurrentLimit instead of setSmartCurrentLimit
m_hookMotor.setCurrentLimit(20); // 20 amps max
        
        // Initialize limit switches
        m_extendedLimitSwitch = new DigitalInput(Constants.HOOK_EXTENDED_LIMIT_SWITCH_PORT);
        m_retractedLimitSwitch = new DigitalInput(Constants.HOOK_RETRACTED_LIMIT_SWITCH_PORT);
        
        // Default to retracted state
        m_isExtended = false;
        m_isRetracting = false;
        
        System.out.println("🪝 HOOK SUBSYSTEM READY TO CONQUER! 🪝");
    }
    
    /**
     * Extends the hook - RELEASE THE KRAKEN!!!
     */
    public void extendHook() {
        if (!isExtended() && !m_extendedLimitSwitch.get()) {
            // Start with slow speed and ramp up!
            long timeSinceStart = System.currentTimeMillis() - m_lastExtendTime;
            
            if (m_lastExtendTime ==0) {
                m_lastExtendTime = System.currentTimeMillis();
                m_currentHookSpeed = Constants.HOOK_EXTEND_MIN_SPEED;
                System.out.println("🪝 STARTING HOOK EXTENSION! 🪝");
            } else if (timeSinceStart < 500) {
                // Ramp up during first 500ms
                double rampProgress = timeSinceStart / 500.0;
                m_currentHookSpeed = Constants.HOOK_EXTEND_MIN_SPEED + 
                    (Constants.HOOK_EXTEND_MAX_SPEED - Constants.HOOK_EXTEND_MIN_SPEED) * rampProgress;
            } else {
                // Full speed after 500ms
                m_currentHookSpeed = Constants.HOOK_EXTEND_MAX_SPEED;
            }
            
            m_hookMotor.set(m_currentHookSpeed);
            m_isRetracting = false;
        } else {
            // We're either fully extended or the limit switch is triggered
            m_hookMotor.set(0);
            
            if (!m_isExtended && m_extendedLimitSwitch.get()) {
                m_isExtended = true;
                System.out.println("✅ HOOK FULLY EXTENDED! READY TO GRAB BARGES!!!");
            }
            
            m_lastExtendTime = 0;
        }
    }
    
    /**
     * Retracts the hook - RETURN TO BASE!!!
     */
    public void retractHook() {
        if (!isRetracted() && !m_retractedLimitSwitch.get()) {
            // ACTIVATE RETRACTION SEQUENCE!
            m_hookMotor.set(-Constants.HOOK_RETRACT_SPEED);
            m_isRetracting = true;
            m_isExtended = false;
        } else {
            // We're either fully retracted or the limit switch is triggered
            m_hookMotor.set(0);
            
            if (m_isRetracting && m_retractedLimitSwitch.get()) {
                m_isRetracting = false;
                System.out.println("✅ HOOK FULLY RETRACTED! READY FOR NEXT DEPLOYMENT!");
            }
        }
        
        m_lastExtendTime = 0;
    }
    
    /**
     * Stops the hook motor - EMERGENCY BRAKE!
     */
    public void stopHook() {
        m_hookMotor.set(0);
        m_lastExtendTime = 0;
        System.out.println("🛑 HOOK STOPPED!");
    }
    
    /**
     * Checks if the hook is fully extended
     * 
     * @return true if hook is at maximum extension
     */
    public boolean isExtended() {
        return m_isExtended || m_extendedLimitSwitch.get();
    }
    
    /**
     * Checks if the hook is fully retracted
     * 
     * @return true if hook is completely retracted
     */
    public boolean isRetracted() {
        return m_retractedLimitSwitch.get();
    }
    
    /**
     * Gets the current hook position
     * Just estimates if we're extended, retracting, or retracted
     * 
     * @return Position as string: "EXTENDED", "RETRACTING", or "RETRACTED"
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
    
    @Override
    public void periodic() {
        // Monitor the hook status and update the dashboard
        SmartDashboard.putBoolean("Hook Extended", isExtended());
        SmartDashboard.putBoolean("Hook Retracted", isRetracted());
        SmartDashboard.putString("Hook Status", getHookPosition());
        SmartDashboard.putNumber("Hook Speed", m_currentHookSpeed);
        SmartDashboard.putNumber("Hook Current", m_hookMotor.getOutputCurrent());
        
        // Safety check - if current is too high, stop motor
        if (m_hookMotor.getOutputCurrent() > Constants.HOOK_MAX_CURRENT) {
            System.out.println("⚠️⚠️⚠️ HOOK CURRENT TOO HIGH! EMERGENCY STOP! ⚠️⚠️⚠️");
            stopHook();
        }
    }
    
    /**
     * Emergency stop all motors - ABORT MISSION!
     */
    public void emergencyStop() {
        m_hookMotor.set(0);
        System.out.println("!!! EMERGENCY STOP ACTIVATED !!!");
        System.out.println("!!! HOOK MOTOR STOPPED !!! ");
    }
}
