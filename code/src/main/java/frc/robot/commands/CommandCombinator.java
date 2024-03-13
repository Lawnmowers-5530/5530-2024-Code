package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LoaderV2;

//credit skyline stole this idea from them
//listen I cant just steal the idea and not name it slightly differently that wouldnt be in the ripoff spirit
public class CommandCombinator {
    Climber climber;
    Intake intake;
    LauncherV2 launcher;
    LoaderV2 loader;
    DistanceSensor distanceSensor;
    DumbLauncherAngle launcherAngle;

    public CommandCombinator(Climber climber, Intake intake, LauncherV2 launcher, LoaderV2 loader,
            DumbLauncherAngle launcherAngle, DistanceSensor distanceSensor) {
        this.climber = climber;
        this.intake = intake;
        this.launcher = launcher;
        this.loader = loader;
        this.distanceSensor = distanceSensor;
        this.launcherAngle = launcherAngle;
    }

    public Command eject() {
        return new ParallelCommandGroup(
                intake.ejectCommand(),
                loader.ejectCommand());
    }

    public Command stopShooterComponents() {
        return new ParallelCommandGroup(
                intake.stopIntakeWheelCommand(),
                loader.stopLoaderCommand(),
                launcher.stopLauncherCommand());
    }

    public Command sourceIntake() {
        return new ParallelCommandGroup(
                new LauncherIntake(distanceSensor, loader, launcher,
                        Constants.LauncherIntakeConstants.threshold, Constants.LauncherIntakeConstants.speed)
                        .until(loader::isLoaded),
                launcherAngle.ampAngleCommand()

        ).andThen(stopShooterComponents());
    }

    public Command groundIntake() {
        return new ParallelCommandGroup(
                intake.intakeWheelCommand(),
                loader.runLoaderCommand())
                .until(loader::isLoaded)
                
                .andThen(stopShooterComponents());
    }

    public Command speakerShot() {
        return launcherAngle.speakerAngleCommand().andThen(new ParallelCommandGroup(
                new VelocityLauncher(
                        launcher,
                        () -> {
                            return Constants.LauncherConstants.LAUNCHER_HIGH_REVS;
                        },
                        () -> {
                            return Constants.LauncherConstants.LAUNCHER_HIGH_REVS
                                    / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
                        }),
                launcherAngle.ampAngleCommand(),
                new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        loader.feedShooterCommand().until(loader::isNotLoaded).andThen(stopShooterComponents()))));
    };

    public Command ampShot() {
        return launcherAngle.speakerAngleCommand().andThen(new ParallelCommandGroup(
                new VelocityLauncher(
                        launcher,
                        () -> {
                            return Constants.LauncherConstants.LAUNCHER_LOW_REVS;
                        },
                        () -> {
                            return Constants.LauncherConstants.LAUNCHER_LOW_REVS
                                    / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
                        }),
                launcherAngle.ampAngleCommand(),
                new SequentialCommandGroup(
                        new WaitCommand(0.5),
                        loader.feedShooterCommand().until(loader::isNotLoaded).andThen(stopShooterComponents()))));
    };
}
