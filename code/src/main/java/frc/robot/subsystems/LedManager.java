package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.GlobalState;
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

public class LedManager extends SubsystemBase {
    LedControllerProxy ledcontroller;

    public LedManager(LedControllerProxy ledcontroller) {
        this.ledcontroller = ledcontroller;
    }

    @Override
    public void periodic() {

    }
}
