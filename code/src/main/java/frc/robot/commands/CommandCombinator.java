package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Subsystems;

//credit skyline stole this idea from them
//listen I cant just steal the idea and not name it slightly differently that wouldnt be in the ripoff spirit
public class CommandCombinator {
	Subsystems subsystems;

	public CommandCombinator(RobotContainer.Subsystems subsystems) {
		this.subsystems = subsystems;
	}

	public Command eject() {
		return new ParallelCommandGroup(
			this.subsystems.intake.ejectCommand(),
			this.subsystems.loader.ejectCommand(),
			this.subsystems.simranIntakeAssist.downAndEject()
		);
	}

	public Command stopShooterComponents() {
		return new ParallelCommandGroup(
			this.subsystems.intake.stopIntakeWheelCommand(),
			this.subsystems.loader.stopLoaderCommand(),
			this.subsystems.launcher.stopLauncherCommand(), 
			this.subsystems.simranIntakeAssist.upAndStop()
		);
	}

	int threshold = Constants.LauncherIntakeConstants.threshold;
    int stage = 0;
    double speed = Constants.LauncherIntakeConstants.speed;
	public Command launcherIntake() {
		return new RunCommand(() -> {
			if (stage == 0 && this.subsystems.distanceSensor.getDistance() < threshold) {
				stage = 1;
			} else if (stage == 1 && this.subsystems.distanceSensor.getDistance() > threshold) {
				stage = 2;
			}
			if (stage == 2) {
				this.subsystems.loader.run(0);
				this.subsystems.launcher.setSpeed(0, 0);
			} else {
				this.subsystems.loader.run(-speed);
				this.subsystems.launcher.setSpeed(-speed, -speed);
			}
		}, this.subsystems.distanceSensor, this.subsystems.loader, this.subsystems.launcher);
	}


	public Command sourceIntake() {
		return this.subsystems.launcherAngle.ampAngleCommand()
			.andThen(
				launcherIntake()
				.until(this.subsystems.distanceSensor::isNotePresent)

			)
			.andThen(stopShooterComponents());
	}

	public Command groundIntake() {
		return this.subsystems.launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelCommandGroup(
					this.subsystems.intake.intakeWheelCommand(),
					this.subsystems.loader.runLoaderCommand().until(this.subsystems.distanceSensor::isNotePresent)
				)
					.until(this.subsystems.distanceSensor::isNotePresent)
					.andThen(stopShooterComponents())
			);
	}

	public Command fullIntake() {
		return this.subsystems.launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelCommandGroup(	
					this.subsystems.intake.intakeWheelCommand(),	
					this.subsystems.simranIntakeAssist.downAndSpin().until(this.subsystems.distanceSensor::isNotePresent),
					this.subsystems.loader.runLoaderCommand().until(this.subsystems.distanceSensor::isNotePresent)
				)
					.until(this.subsystems.distanceSensor::isNotePresent)
					.andThen(stopShooterComponents())
			);
	}

	public Command autoIntake() {
		return this.subsystems.launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelCommandGroup(	
				this.subsystems.intake.intakeWheelCommand(),	
				this.subsystems.simranIntakeAssist.downAndSpin().withTimeout(1),
				this.subsystems.loader.runLoaderCommand().until(this.subsystems.distanceSensor::isNotePresent)
					
				)
					.until(this.subsystems.distanceSensor::isNotePresent).withTimeout(3)
					.andThen(stopShooterComponents())
			);
	}

	public Command speakerShot() {
		return this.subsystems.launcherAngle.ampAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						this.subsystems.loader.feedShooterCommand().until(this.subsystems.distanceSensor::isNoteNotPresent)
					),
					this.subsystems.launcher.speakerLauncherCommand()
				)
			)
			.andThen(logFinish("speakerShot")).andThen(stopShooterComponents());
	};

	public Command speakerFarShot() { //53 inches from base
		return this.subsystems.launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						this.subsystems.loader.feedShooterCommand().until(this.subsystems.distanceSensor::isNoteNotPresent)
					),
				this.subsystems.launcher.speakerLauncherCommand()
				)
			)
			.andThen(stopShooterComponents());
	};

	public Command lobShot() {
		return this.subsystems.launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						this.subsystems.loader.feedShooterCommand().until(this.subsystems.distanceSensor::isNoteNotPresent)
					),
				this.subsystems.launcher.lobLauncherCommand()
				)
			)
			.andThen(stopShooterComponents());
	}

	public Command ampShot() {
		return this.subsystems.launcherAngle.ampAngleCommand()

			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						this.subsystems.loader.feedShooterCommand().until(this.subsystems.distanceSensor::isNoteNotPresent)
					),
					this.subsystems.launcher.ampLauncherCommand()
				)
			)
			.andThen(stopShooterComponents());
	};

	public Command ampShotAssist() {
		return new ParallelDeadlineGroup(
			this.subsystems.launcherAngle.ampAngleCommand(), 
			new WaitCommand(0.75), 
			this.subsystems.ampAssist.up()
		)
		.andThen(
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(
					new WaitCommand(0.75),
					this.subsystems.loader.feedShooterCommand().until(this.subsystems.distanceSensor::isNoteNotPresent)
				),
				this.subsystems.launcher.ampLauncherCommand()
			)
		)
		.andThen(new WaitCommand(0.75), this.subsystems.ampAssist.down(), stopShooterComponents());
	};

	public Command autoSpeakerLauncher(){
		return new ParallelCommandGroup(
			this.subsystems.launcher.speakerLauncherCommand(),
			this.subsystems.launcherAngle.ampAngleCommand()
		);
	}

	public Command autoSpeakerFarLauncher(){
		return new ParallelCommandGroup(
			this.subsystems.launcher.speakerLauncherCommand(),
			this.subsystems.launcherAngle.speakerAngleCommand()
		);
	}

	private Command logFinish(String cmdName) {
		return new InstantCommand(
				() -> {
					System.out.println(cmdName + " finished");
				}, new Subsystem[] {});
	}

	// public Command ExternalIntakeOn() {
	// 	return new SequentialCommandGroup(	
	// 	externalIntake.setPivotCommand(Position.DOWN).until(externalIntake::ready),
	// 	externalIntake.externalIntakeWheelCommand()
	// 	);
	// }

	// public Command ExternalIntakeOff() {
	// 	return new SequentialCommandGroup(
	// 		externalIntake.stopExternalIntakeWheelCommand(),
	// 		externalIntake.setPivotCommand(Position.UP)
	// 			.until(externalIntake::ready)
	// 	);
	// }
}
