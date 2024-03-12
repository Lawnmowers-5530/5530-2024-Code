package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;

    public LoaderV2(int motorLeft, int motorRight, boolean reversed, DistanceSensor distanceSensor) {
        super(motorLeft, motorRight, reversed);
        this.distanceSensor = distanceSensor;
    }
}
