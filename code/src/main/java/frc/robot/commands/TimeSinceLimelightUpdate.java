package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.StaticLimeLight;

public class TimeSinceLimelightUpdate extends Command {
    private final Timer timer;
    private final double threshold = 2.0; // Adjust this threshold as needed

    public TimeSinceLimelightUpdate() {
        this.addRequirements(new Subsystem[]{});
        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        if (StaticLimeLight.getValidTarget()) { // Replace isValueTrue() with your actual method
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(threshold);
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}