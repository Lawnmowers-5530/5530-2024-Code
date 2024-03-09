package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DumbLauncherAngle;

public class AngleLauncher extends Command {
    public enum Angle {
        UP, DOWN, RELAXED
    }
    Angle state;
    DumbLauncherAngle launcherAngle;
    public AngleLauncher(DumbLauncherAngle launcherAngle, Angle state) {
        this.state = state;
        this.launcherAngle = launcherAngle;
        addRequirements(launcherAngle);
    }

    @Override
    public void execute() {
        switch (state) {
            case UP: {
                launcherAngle.forceUp();
            }
            case DOWN: {
                launcherAngle.forceDown();
            }
            case RELAXED:  {
                launcherAngle.relax();
            }}
    }

    @Override
    public boolean isFinished() {
        switch (state) {
            case UP: {
                if (launcherAngle.getEncoderMeasurement() > Constants.LauncherAngleConstants.HIGH_ANGLE_MEASUREMENT) {
                    return true;
                }
            }
            case DOWN: {
                if (launcherAngle.getEncoderMeasurement() < Constants.LauncherAngleConstants.LOW_ANGLE_MEASUREMENT) {
                    return true;
                }
            }
            case RELAXED: {
                return true;
            }
            default: {
                return true;
            }
        }
    }

        
}
