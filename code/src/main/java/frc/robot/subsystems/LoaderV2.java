package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.data.GlobalState;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;
    GlobalState state;

    public LoaderV2(int motorLeft, int motorRight, boolean reversed, DistanceSensor distanceSensor) {
        super(motorLeft, motorRight, reversed);
        this.distanceSensor = distanceSensor;
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
            GlobalState.noteLoaded = true;
        } else {
            GlobalState.noteLoaded = false;
        }
    }
}
