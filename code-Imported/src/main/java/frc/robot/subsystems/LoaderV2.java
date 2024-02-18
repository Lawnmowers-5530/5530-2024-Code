package frc.robot.subsystems;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;
    public LoaderV2(int motor, boolean reversed, DistanceSensor distanceSensor) {
        super(motor, reversed);
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
