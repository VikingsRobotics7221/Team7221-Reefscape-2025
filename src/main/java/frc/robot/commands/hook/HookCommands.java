package frc.robot.commands.hook;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HookSubsystem;

/**
 * HookCommands - Command classes for controlling the barge hook mechanism
 * 
 * This file contains a collection of commands that control the robot's hook system
 * for barge manipulation during the endgame phase of FRC Reefscape.
 * 
 * The hook system uses a linear actuator with a J-hook end effector to:
 * 1. Extend vertically to reach the barge
 * 2. Securely attach to the barge
 * 3. Retract to pull the barge in the desired direction
 * 
 * System connections:
 * - These commands require the HookSubsystem to function
 * - They are activated through button bindings defined in Robot.java
 * - They use limit switches to detect when the hook reaches its limits
 */
public class HookCommands {
    
    /**
     * ExtendHookCommand - Extends the hook until it reaches its limit
     * 
     * This command extends the hook upward until the limit switch is triggered,
     * indicating the hook has reached its maximum extension.
     */
    public static class ExtendHookCommand extends Command {
        private final HookSubsystem m_hookSubsystem;
        
        public ExtendHookCommand(HookSubsystem hookSubsystem) {
            m_hookSubsystem = hookSubsystem;
            addRequirements(hookSubsystem);
        }
        
        @Override
        public void initialize() {
            System.out.println("Extending hook...");
        }
        
        @Override
        public void execute() {
            m_hookSubsystem.extendHook();
        }
        
        @Override
        public void end(boolean interrupted) {
            m_hookSubsystem.stopHook();
            if (interrupted) {
                System.out.println("Hook extension interrupted");
            } else {
                System.out.println("Hook fully extended");
            }
        }
        
        @Override
        public boolean isFinished() {
            return m_hookSubsystem.isExtended();
        }
    }
    
    /**
     * RetractHookCommand - Retracts the hook until it reaches its limit
     * 
     * This command retracts the hook downward until the limit switch is triggered,
     * indicating the hook has fully retracted. When used after hooking onto the barge,
     * this command will pull the barge with the robot.
     */
    public static class RetractHookCommand extends Command {
        private final HookSubsystem m_hookSubsystem;
        
        public RetractHookCommand(HookSubsystem hookSubsystem) {
            m_hookSubsystem = hookSubsystem;
            addRequirements(hookSubsystem);
        }
        
        @Override
        public void initialize() {
            System.out.println("Retracting hook...");
        }
        
        @Override
        public void execute() {
            m_hookSubsystem.retractHook();
        }
        
        @Override
        public void end(boolean interrupted) {
            m_hookSubsystem.stopHook();
            if (interrupted) {
                System.out.println("Hook retraction interrupted");
            } else {
                System.out.println("Hook fully retracted");
            }
        }
        
        @Override
        public boolean isFinished() {
            return m_hookSubsystem.isRetracted();
        }
    }
    
    /**
     * HookCycleCommand - Runs a complete hook cycle
     * 
     * This command sequence executes a full hook operation:
     * 1. Extend the hook
     * 2. Wait for barge connection (or a set time delay)
     * 3. Retract the hook, pulling the barge
     * 
     * Useful for automatic barge capture during endgame.
     */
    public static class HookCycleCommand extends SequentialCommandGroup {
        public HookCycleCommand(HookSubsystem hookSubsystem) {
            addCommands(
                // Start with announcement
                new InstantCommand(() -> {
                    System.out.println("Starting full hook cycle");
                }),
                
                // Extend the hook
                new ExtendHookCommand(hookSubsystem),
                
                // Wait for barge connection (or a set time)
                new InstantCommand(() -> System.out.println("Hook extended - waiting for barge connection")),
                new WaitCommand(2.0),  // In competition, replace with driver confirmation
                
                // Retract with the barge
                new RetractHookCommand(hookSubsystem),
                
                // Finish with success message
                new InstantCommand(() -> {
                    System.out.println("Hook cycle complete");
                })
            );
        }
    }
    
    /**
     * Emergency stop command for the hook system
     * 
     * @param hookSubsystem The hook subsystem to stop
     * @return A command that immediately stops hook movement
     */
    public static Command emergencyStopHook(HookSubsystem hookSubsystem) {
        return Commands.runOnce(() -> {
            hookSubsystem.emergencyStop();
            System.out.println("HOOK EMERGENCY STOP ACTIVATED");
        });
    }
}
