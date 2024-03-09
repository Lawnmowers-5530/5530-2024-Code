package frc.robot.subsystems;

import frc.robot.Constants;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;

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
}
