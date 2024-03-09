package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class StopCommand<T extends Subsystem> extends Command {
    public T subsystem;
    public StopCommand(T subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {};

    @Override
    public boolean isFinished() {
        return true;
    }

}
