package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoaderV2;

public class IntakeOffFloor extends Command {
    Intake intake;
    LoaderV2 loader;
    double speed;
    double cutoff;
    boolean finished = false;
    public IntakeOffFloor(Intake intake, LoaderV2 loader, double speed, double cutoff) {
        this.speed = speed;
        this.cutoff = cutoff;
        this.intake = intake;
        this.loader = loader;
        addRequirements(intake);
        addRequirements(loader);
    }

    @Override
    public void execute() {
        intake.run(speed);
        finished = loader.runUntilBeamBreak(speed, speed);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
        loader.run(0);
    }

    public class IntakeOffFloorFactory {
        Intake intake;
        LoaderV2 loader;
        double speed;
        double cutoff;
        public IntakeOffFloorFactory(Intake intake, LoaderV2 loader, double speed, double cutoff) {
            this.intake = intake;
            this.loader = loader;
            this.speed = speed;
            this.cutoff = cutoff;
        }

        public IntakeOffFloor create() {
            return new IntakeOffFloor(intake, loader, speed, cutoff);
        }
    }
}
