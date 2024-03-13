package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LedController.PatternType;
import frc.robot.subsystems.LedController.fixedPalattePatternType;
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

public class LedController_MultiAccess extends SubsystemBase {
    LedController controller;
    private static ArrayList<LedControllerCommand> queue = new ArrayList<>();

    public LedController_MultiAccess(LedController controller) {
        this.controller = controller;
    }

    /** 
     * A proxy for the real {@link LedController} to be used by other commands and subsystems at the same time
     * @return A Proxy to the real {@link LedController}, use the setPattern method to set the pattern to be displayed
    */
    public LedControllerProxy getController() {
        return new LedControllerProxy();
    }

    public class LedControllerProxy {
        /**
         * sets the pattern to be displayed
         * the pattern displayed will be the pattern with the highest priority submitted last
         * @param patternType The pattern type to be displayed
         * @param priority The priority of the pattern, higher priority patterns will be displayed instead of lower priority patterns. Does not accept values below 0.
         */

        public void setPattern(PatternType patternType, int priority) {
            if (priority < 0) {
                priority = 0;
            }
            LedControllerCommand command = new LedControllerCommand();
            command.patternType = patternType;
            command.priority = priority;
            synchronized (LedController_MultiAccess.queue) {
                LedController_MultiAccess.queue.add(command);
            }
        }
    }
    // info for a pattern to be set
    public class LedControllerCommand {
        int priority;
        PatternType patternType;
    }
    /** updates the controller with the highest priority pattern in the queue */
    public void periodic() {
        int highestPriority = -1;
        LedControllerCommand highestPriorityCommand = null;
        synchronized (LedController_MultiAccess.queue) {
            for (LedControllerCommand command : queue) {
                if (command.priority >= highestPriority) {
                    highestPriority = command.priority;
                    highestPriorityCommand = command;
                }
            }
        }
        if (highestPriorityCommand != null) {
            controller.setPattern(highestPriorityCommand.patternType);
            synchronized (LedController_MultiAccess.queue) {
                queue.clear();
            }
        }
    }

    public Command LedControllerCommand(PatternType patternType, int priority) {
        return this.runOnce(
        () -> {
          LedControllerProxy ledProxy = this.getController();
          ledProxy.setPattern(fixedPalattePatternType.ColorWavesOcean, 1);
        }
        );
    }
}