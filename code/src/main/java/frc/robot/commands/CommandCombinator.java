package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherAngle;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LoaderV2;

//credit skyline stole this idea from them
//listen I cant just steal the idea and not name it slightly differently that wouldnt be in the ripoff spirit
public class CommandCombinator {
    Climber climber;
    Intake intake;
    LauncherV2 launcher;
    LoaderV2 loader;
    LauncherAngleV2 launcherAngle;

    public CommandCombinator(Climber climber, Intake intake, LauncherV2 launcher, LoaderV2 loader) {
        this.climber = climber;
        this.intake = intake;
        this.launcher = launcher;
        this.loader = loader;
    }

    public Command intakeOffFloorCommand() {
        return new SequentialCommandGroup(

        );
    };
}
