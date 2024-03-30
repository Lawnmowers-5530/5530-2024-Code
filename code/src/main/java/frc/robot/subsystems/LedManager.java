package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LedConstants.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

public class LedManager extends SubsystemBase {
    LedControllerProxy ledcontroller;

    public LedManager(LedControllerProxy ledcontroller) {
        this.ledcontroller = ledcontroller;
    }

    @Override
    public void periodic() { /** 
        if (!GlobalState.noteLoaded && GlobalState.armReady) {
            ledcontroller.setPattern(Patterns.NO_NOTE_ARM_READY, Priorities.MEDIUM);
        }
        if (GlobalState.noteLoaded) {
            ledcontroller.setPattern(Patterns.NOTE_LOADED, Priorities.HIGH);
        }
        if (GlobalState.noteLoaded && GlobalState.inSpeakerZone) {
            ledcontroller.setPattern(Patterns.NOTE_LOADED_IN_ZONE, Priorities.MEDIUM);
        }
        if (!GlobalState.noteLoaded && !GlobalState.armReady) {
            ledcontroller.setPattern(Patterns.NO_NOTE_ARM_NOT_READY, Priorities.MEDIUM);
        }**/
    }
    

    /**
    *A command that sets the pattern to be displayed based on the state of the robot
    *@param groundIntakeRunningAmpAngle A boolean supplier that returns true if the ground intake is running and the launcher is at the amp angle
    *@param readyToIntakeFromSource A boolean supplier that returns true if the robot is ready to intake from the source
    *@param readyToShoot A boolean supplier that returns true if the robot is ready to shoot
    *@param noteLoaded A boolean supplier that returns true if the note is loaded
    *@param slowMode A boolean supplier that returns true if the robot is in slow mode
    *@return A command that sets the pattern to be displayed based on the state of the robot
    */
    public Command LedControllingCommand(RobotContainer.StateSuppliers stateSuppliers) {
        return new RunCommand(
            () -> {
                
                if (stateSuppliers.groundIntakeRunningAmpAngle.getAsBoolean())
                {
                    ledcontroller.setPattern(groundIntakeAndAmpAnglePattern, groundIntakeAndAmpAnglePriority);
                }
                else if (stateSuppliers.readyToIntakeFromSource.getAsBoolean())
                {
                    ledcontroller.setPattern(intakeFromSourceReadyPattern, intakeFromSourceReadyPriority);
                }
                else if (stateSuppliers.noteLoaded.getAsBoolean())
                {
                    ledcontroller.setPattern(noteLoadedPattern, noteLoadedPriority);
                }
                else if (stateSuppliers.slowMode.getAsBoolean())
                {
                    ledcontroller.setPattern(slowModePattern, slowModePriority);
                }
                else
                {
                    ledcontroller.setPattern(defaultPattern, defaultPriority);
                }
            }, this);
    }
}
