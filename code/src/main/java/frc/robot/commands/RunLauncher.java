package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LauncherV2;

public class RunLauncher extends Command {
    LauncherV2 launcher;
    double speed;
    double diffPercent;
    public RunLauncher(LauncherV2 launcher, double speed, double diffPercent) {
        this.speed = speed;
        this.diffPercent = diffPercent;
        addRequirements(launcher);
    }

    @Override
    public void execute() {
        launcher.setVelocity(speed, speed - (speed * diffPercent));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        launcher.setVelocity(0, 0);
    }
}
