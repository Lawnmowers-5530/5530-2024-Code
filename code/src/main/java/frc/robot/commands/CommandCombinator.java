package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.LimelightHelpers;
import frc.lib.Shot;
import frc.lib.ShotCalculator;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.LauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;

//credit skyline stole this idea from them
//listen I cant just steal the idea and not name it slightly differently that wouldnt be in the ripoff spirit
public class CommandCombinator {
	Climber climber;
	Intake intake;
	LauncherV2 launcher;
	LoaderV2 loader;
	DistanceSensor distanceSensor;
	LauncherAngle launcherAngle;
	Swerve swerve;

	public CommandCombinator(Climber climber, Intake intake, LauncherV2 launcher, LoaderV2 loader,
			LauncherAngle launcherAngle, DistanceSensor distanceSensor, Swerve swerve) {
		this.climber = climber;
		this.intake = intake;
		this.launcher = launcher;
		this.loader = loader;
		this.distanceSensor = distanceSensor;
		this.launcherAngle = launcherAngle;
		this.swerve = swerve;
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
		return launcherAngle.ampAngleCommand()
			.andThen(
				new LauncherIntake(distanceSensor, loader, launcher,
						Constants.LauncherIntakeConstants.threshold,
						Constants.LauncherIntakeConstants.speed)
				.until(loader::isLoaded)

			)
			.andThen(stopShooterComponents());
	}

	public Command groundIntake() {
		return launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelCommandGroup(
					intake.intakeWheelCommand(),
					loader.runLoaderCommand()
				)
					.until(loader::isLoaded)
					.andThen(stopShooterComponents())
			);
	}

	public Command speakerShot() {
		return launcherAngle.ampAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						loader.feedShooterCommand().until(loader::isNotLoaded)
					),
					launcher.speakerLauncherCommand()
				)
			)
			.andThen(logFinish("speakerShot")).andThen(stopShooterComponents());
	};

	public Command speakerFarShot() { //53 inches from base
		return launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						loader.feedShooterCommand().until(loader::isNotLoaded)
					),
				launcher.speakerLauncherCommand()
				)
			)
			.andThen(stopShooterComponents());
	};

	public Command ampShot() {
		return launcherAngle.ampAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						loader.feedShooterCommand().until(loader::isNotLoaded)
					),
					launcher.ampLauncherCommand()
				)
			)
			.andThen(stopShooterComponents());
	};

	private Command logFinish(String cmdName) {
		return new InstantCommand(
				() -> {
					System.out.println(cmdName + " finished");
				}, new Subsystem[] {});
	}


	//Shoot anywhere command based on swerve current pose
	public Command shootPoseCommand() {
		return new RunCommand(
      () -> {
        Pose2d currentPose = swerve.getPose();

        // find field oriented vector of robot
        Vector2D robotVector = swerve.getFieldRelativeSpeeds();

        // find distance and angle to target
        double distToTarget = Constants.targetTranslation.getDistance(currentPose.getTranslation());
        double angleToTarget = ShotCalculator.angleToTarget(currentPose);

        //use shooter library to calculate final shot vector
        Shot shot = ShotCalculator.vecFinal(robotVector, distToTarget, angleToTarget);

		if(shot != null){
        // shoot the calculated shot
        double leftSpeed = 0.4;
        double rightSpeed = 0.4;

        launcher.setVelocity(leftSpeed, rightSpeed);
        launcherAngle.setAngle(shot.getThetaDeg());

        loader.run(Constants.LauncherConstants.loaderShotSpeed);

        swerve.rotateToAngle(Math.toDegrees(shot.getPhiDeg()));
		}else{
			System.out.println("Shot is not possible");
		}
      }, new Subsystem[] { launcher, launcherAngle, swerve });
}
}
