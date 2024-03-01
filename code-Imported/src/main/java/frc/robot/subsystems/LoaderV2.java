package frc.robot.subsystems;

import frc.robot.Constants;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;

    public LoaderV2(int motorLeft, int motorRight, boolean reversed, DistanceSensor distanceSensor) {
        super(motorLeft, motorRight, reversed);
        this.distanceSensor = distanceSensor;
    }

    public boolean runUntilBeamBreak(double speed, double cutoffDistance, Intake intake) {
        System.out.println("running beam break");
            if (distanceSensor.getDistance() < cutoffDistance) {
                intake.run(0);
                System.out.println("ended");
                run(0);
                return true;
            } else {
                intake.run(Constants.IntakeConstants.intakeSpeed);
                run(speed);
                return false;
            }
        }
}
