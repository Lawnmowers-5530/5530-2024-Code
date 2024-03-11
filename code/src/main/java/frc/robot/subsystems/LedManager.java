package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.Patterns;
import frc.robot.Constants.LEDConstants.Priorities;
import frc.robot.data.GlobalState;
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

public class LedManager extends SubsystemBase {
    LedControllerProxy ledcontroller;

    public LedManager(LedControllerProxy ledcontroller) {
        this.ledcontroller = ledcontroller;
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
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
        }
    }
}
