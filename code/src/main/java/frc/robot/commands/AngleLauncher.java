package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.data.GlobalState;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.DumbLauncherAngle.Angle;

public class AngleLauncher extends Command {
    Angle state;
    DumbLauncherAngle launcherAngle;
    public AngleLauncher(DumbLauncherAngle launcherAngle, Angle state) {
        this.state = state;
        this.launcherAngle = launcherAngle;
        addRequirements(launcherAngle);
    }

    @Override
    public void execute() {
        launcherAngle.setState(state);
    }

    @Override
    public boolean isFinished() {
        if (GlobalState.armReady) {
            return true;
        } else {
            return false;
        }
    }

    public class AngleLauncherFactory {
        DumbLauncherAngle launcherAngle;
        Angle state;
        public AngleLauncherFactory(DumbLauncherAngle launcherAngle, Angle state) {
            this.launcherAngle = launcherAngle;
            this.state = state;
        }

        public AngleLauncher create() {
            return new AngleLauncher(launcherAngle, state);
        }
    }
}
