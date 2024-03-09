package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoaderV2;

public class RunLoader extends Command {
    LoaderV2 loader;
    double speed;
    public RunLoader(LoaderV2 loader, double speed) {
        this.speed = speed;
        this.loader = loader;
        addRequirements(loader);
    }

    @Override
    public void execute() {
        loader.run(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        loader.run(0);
    }
}
