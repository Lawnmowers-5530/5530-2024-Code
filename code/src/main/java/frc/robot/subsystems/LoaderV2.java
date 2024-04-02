package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.data.GlobalState;
import static frc.robot.Constants.LoaderConstants.*;
import static frc.robot.Constants.*;

public class LoaderV2 extends Loader {
    GlobalState state;

    public LoaderV2() {
        super();
    }

    public Command runLoaderCommand() {
        return new RunCommand(
            () -> {
                this.run(loaderSpeed);
            }, this);
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
