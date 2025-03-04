/*
 * ================================================================
 *  _    _  ____   ____  _  __     _____ __  __ _____   _____ !
 * | |  | |/ __ \ / __ \| |/ /    / ____|  \/  |  __ \ / ____|!
 * | |__| | |  | | |  | | ' /    | |    | \  / | |  | | (___  !
 * |  __  | |  | | |  | |  <     | |    | |\/| | |  | |\___ \ !
 * | |  | | |__| | |__| | . \    | |____| |  | | |__| |____) |!
 * |_|  |_|\____/ \____/|_|\_\    \_____|_|  |_|_____/|_____/ !
 *                                                             !
 * ================================================================
 * 
 * TEAM 7221 HOOK COMMANDS - FOR MAXIMUM BARGE DOMINATION
 * 
 *   |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|
 */

package frc.robot.commands.hook;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.HookSubsystem;

/**
 * Hook commands for controlling our epic barge hook system
 * 
 * We've got awesome commands to:
 * - Extend the hook
 * - Retract the hook
 * - Run the full grab sequence
 */
public class HookCommands {
    
    /**
     * Command to extend the hook
     */
    public static class ExtendHookCommand extends Command {
        private final HookSubsystem m_hookSubsystem;
        
        public ExtendHookCommand(HookSubsystem hookSubsystem) {
            m_hookSubsystem = hookSubsystem;
            addRequirements(hookSubsystem);
        }
        
        @Override
        public void initialize() {
            System.out.println(">> EXTENDING HOOK - RELEASE THE KRAKEN! <<");
        }
        
        @Override
        public void execute() {
            m_hookSubsystem.extendHook();
        }
        
        @Override
        public void end(boolean interrupted) {
            m_hookSubsystem.stopHook();
            if (interrupted) {
                System.out.println(">> HOOK EXTENSION INTERRUPTED!");
            } else {
                System.out.println(">> HOOK FULLY EXTENDED!");
            }
        }
        
        @Override
        public boolean isFinished() {
            return m_hookSubsystem.isExtended();
        }
    }
    
    /**
     * Command to retract the hook
     */
    public static class RetractHookCommand extends Command {
        private final HookSubsystem m_hookSubsystem;
        
        public RetractHookCommand(HookSubsystem hookSubsystem) {
            m_hookSubsystem = hookSubsystem;
            addRequirements(hookSubsystem);
        }
        
        @Override
        public void initialize() {
            System.out.println(">> RETRACTING HOOK - COMING HOME!");
        }
        
        @Override
        public void execute() {
            m_hookSubsystem.retractHook();
        }
        
        @Override
        public void end(boolean interrupted) {
            m_hookSubsystem.stopHook();
            if (interrupted) {
                System.out.println(">> HOOK RETRACTION INTERRUPTED!");
            } else {
                System.out.println(">> HOOK FULLY RETRACTED!");
            }
        }
        
        @Override
        public boolean isFinished() {
            return m_hookSubsystem.isRetracted();
        }
    }
    
    /**
     * Full hook cycle command - extends, waits for barge connection, then retracts
     */
    public static class HookCycleCommand extends SequentialCommandGroup {
        public HookCycleCommand(HookSubsystem hookSubsystem) {
            addCommands(
                // Start with cool announcement
                new InstantCommand(() -> {
                    System.out.println("");
                    System.out.println(">>>>>> INITIATING FULL HOOK CYCLE! >>>>>>");
                    System.out.println(">>>>>> PREPARE FOR BARGE CAPTURE! >>>>>>");
                    System.out.println("");
                }),
                
                // Extend the hook
                new ExtendHookCommand(hookSubsystem),
                
                // Wait for barge connection (operator will press button to continue)
                new InstantCommand(() -> {
                    System.out.println(">> WAITING FOR BARGE CONNECTION!");
                    System.out.println(">> PRESS CONFIRM WHEN CONNECTED!");
                }),
                
                // This would normally be replaced by a button press to continue
                // For testing purposes, we'll just wait 2 seconds
                new WaitCommand(2.0),
                
                // Retract with the barge
                new RetractHookCommand(hookSubsystem),
                
                // Finish with success message
                new InstantCommand(() -> {
                    System.out.println("");
                    System.out.println(">>>>>> BARGE CAPTURED SUCCESSFULLY! >>>>>>");
                    System.out.println(">>>>>> HOOK CYCLE COMPLETE! >>>>>>");
                    System.out.println("");
                })
            );
        }
    }
    
    /**
     * Emergency stop command for the hook system
     */
    public static Command emergencyStopHook(HookSubsystem hookSubsystem) {
        return Commands.runOnce(() -> {
            hookSubsystem.emergencyStop();
            System.out.println("!!! HOOK EMERGENCY STOP ACTIVATED !!!");
        });
    }
}
