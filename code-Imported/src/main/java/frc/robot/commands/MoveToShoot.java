package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Vector2D;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class MoveToShoot extends Command {
    private double angle;
    private Swerve swerve;
    private boolean isFinished = false;
    private Pose2d goalPose;

    public MoveToShoot(Swerve swerve, Pose2d currentPose) {
        this.swerve = swerve;
        this.goalPose = currentPose; //TODO

    }

    @Override
    public void execute() {

        isFinished = swerve.rotateToAngle(angle) && swerve.getPose().getTranslation().getDistance(Constants.targetTranslation) <= Constants.shotDistance;

        if (swerve.getPose().getTranslation().getDistance(Constants.targetTranslation) <= Constants.shotDistance) {
            swerve.drive(new Vector2D(0, 0, false), 0, false);
        } else {
            swerve.drive(new Vector2D(0, 0, false), 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Vector2D(0, 0, false), 0, false);
    }
}