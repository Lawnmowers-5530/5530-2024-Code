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

	
	//Eject Ring
	public Command eject() {
		return new ParallelCommandGroup(
			this.subsystems.intake.ejectCommand(),
			this.subsystems.loader.ejectCommand(),
			this.subsystems.simranIntakeAssist.downAndEject()
		);
	}


	//Stop all Componets, and retract everything
	public Command stopShooterComponents() {
		return new ParallelCommandGroup(
			this.subsystems.intake.stopIntakeWheelCommand(),
			this.subsystems.loader.stopLoaderCommand(),
			this.subsystems.launcher.stopLauncherCommand(), 
			this.subsystems.simranIntakeAssist.upAndStop(),
			this.subsystems.ampAssist.down()
		);
	}


	//Distance Sensor Fall back, stops all componets and brings ring back into gate wheels so it is no longer touching fly wheels
	public Command manualIntakeStop(){
		return new SequentialCommandGroup(
			stopShooterComponents(),
			this.subsystems.loader.ejectCommand(),
			new WaitCommand(0.25), //subject to change
			stopShooterComponents()
		);
	}

	//Intake Fall back, intakes from source above

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

	//Intakes from ground using ONLY internal
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

	
	//Intakes from ground using external and internal
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

	//FOR AUTON, Intakes from ground using external and internal, but times out after 3 seconds
	public Command autonIntake() {
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

	//FOR AUTON, feeds the already spun up shooter and turns of the componets
	public Command feedAndOff(){
		return new SequentialCommandGroup(
			this.subsystems.loader.feedShooterCommand().until(this.subsystems.distanceSensor::isNoteNotPresent).withTimeout(0.1),//TIME subject to change
			stopShooterComponents()
		);
	}

	//FOR AUTON, spins up the shooter and angles the launcher to shoot
	public Command spinAndAngle(){
		return new ParallelCommandGroup(
		this.subsystems.launcherAngle.ampAngleCommand(),
		this.subsystems.launcher.speakerLauncherCommand()
		);
	}

	
	//Shoots into the speaker, from the close angle and high rpm
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

	
	//Shoots into the speaker, from the far angle and high rpm
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

	//Shoots from the centerline to into wing USED during auto - CURRENTLY ILLEGAL NEEDS TO CHANGE
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

	//Shoots into amp WITHOUT amp assist arm
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

	
	//shoots into amp WITH amp assist arm
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

	
	private Command logFinish(String cmdName) {
		return new InstantCommand(
				() -> {
					System.out.println(cmdName + " finished");
				}, new Subsystem[] {});
	}

	
}
