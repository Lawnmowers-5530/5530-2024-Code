package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.data.GlobalState;
import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.*;

public class LoaderV2 extends Loader {
    DistanceSensor distanceSensor;
    GlobalState state;

    public LoaderV2() {
        super();
    }

    @Deprecated
    public boolean isLoaded() {
        return distanceSensor.checkBeamBreak(loaderCutoffDistance);
    }

    @Deprecated
    public boolean isNotLoaded() {
        return !isLoaded();
    }

    public Command runLoaderCommand() {
        return new RunCommand(
            () -> {
                if (!isLoaded()) {
                    this.run(loaderSpeed);
                } else {
                    this.run(0);
                }
            });
    }

    public Command feedShooterCommand() {
        return new RunCommand(
            () -> {
              this.run(loaderSpeed);
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
                this.run(ejectSpeed);
            });
    }
}
