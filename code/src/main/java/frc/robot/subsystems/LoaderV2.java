package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.LedController.SolidColorType;
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;
    LedControllerProxy leds;

    public LoaderV2(int motorLeft, int motorRight, boolean reversed, DistanceSensor distanceSensor, LedControllerProxy leds) {
        super(motorLeft, motorRight, reversed);
        this.distanceSensor = distanceSensor;
        this.leds = leds;
    }

    public boolean runUntilBeamBreak(double speed, double cutoffDistance) {
            if (distanceSensor.getDistance() < cutoffDistance) {
                run(0);
                return true;
            } else {
                run(speed);
                return false;
            }
    }

    @Override
    public void periodic() {
        if (distanceSensor.getDistance() < Constants.LoaderConstants.loaderCutoffDistance) {
            leds.setPattern(SolidColorType.Blue, Constants.LoaderConstants.NOTE_LOADED_PRIORITY);
        }
    }
}
