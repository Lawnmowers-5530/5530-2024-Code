package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.data.GlobalState;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;
    GlobalState state;

    public LoaderV2(int motorLeft, int motorRight, boolean reversed, DistanceSensor distanceSensor) {
        super(motorLeft, motorRight, reversed);
        this.distanceSensor = distanceSensor;
    }

    public boolean isLoaded() {
        return distanceSensor.checkBeamBreak(Constants.LoaderConstants.loaderCutoffDistance);
    }

    public boolean isNotLoaded() {
        return !isLoaded();
    }

    public Command runLoaderCommand() {
        return new RunCommand(
            () -> {
                if (!isLoaded()) {
                    this.run(Constants.LoaderConstants.loaderSpeed);
                } else {
                    this.run(0);
                }
            });
    }

    public Command feedShooterCommand() {
        return new RunCommand(
            () -> {
              this.run(Constants.LoaderConstants.loaderSpeed);
            }, this);
    }

    public Command stopLoaderCommand() {
        return this.runOnce(
            () -> {
                this.run(0);
            });
    }

    public Command ejectCommand() {
        return this.runOnce(
            () -> {
                this.run(Constants.ejectSpeed);
            });
    }
}
