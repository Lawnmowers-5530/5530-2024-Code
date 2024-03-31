package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AmpAssist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.SimranIntakeAssist;

//credit skyline stole this idea from them
//listen I cant just steal the idea and not name it slightly differently that wouldnt be in the ripoff spirit
public class CommandCombinator {
	Climber climber;
	Intake intake;
	LauncherV2 launcher;
	LoaderV2 loader;
	DistanceSensor distanceSensor;
	DumbLauncherAngle launcherAngle;
	AmpAssist ampAssist;
	SimranIntakeAssist simranIntakeAssist;

	public CommandCombinator(Climber climber, Intake intake, LauncherV2 launcher, LoaderV2 loader,
			DumbLauncherAngle launcherAngle, DistanceSensor distanceSensor, AmpAssist ampAssist, SimranIntakeAssist simranIntakeAssist) {
		this.climber = climber;
		this.intake = intake;
		this.launcher = launcher;
		this.loader = loader;
		this.distanceSensor = distanceSensor;
		this.launcherAngle = launcherAngle;
		this.ampAssist = ampAssist;
		this.simranIntakeAssist = simranIntakeAssist;
		
	}

	public Command eject() {
		return new ParallelCommandGroup(
				intake.ejectCommand(),
				loader.ejectCommand(),
				simranIntakeAssist.downAndEject()
				);
	}

	public Command stopShooterComponents() {
		return new ParallelCommandGroup(
				
				intake.stopIntakeWheelCommand(),
				loader.stopLoaderCommand(),
				launcher.stopLauncherCommand(), 
				simranIntakeAssist.upAndStop()
				);
	}

	public Command manualIntakeStop(){
		return new SequentialCommandGroup(
			stopShooterComponents(),
			loader.ejectCommand(),
			new WaitCommand(0.25), //subject to change
			stopShooterComponents()
		);
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

	public Command fullIntake() {
		return launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelCommandGroup(	
				intake.intakeWheelCommand(),	
				simranIntakeAssist.downAndSpin().until(loader::isLoaded),
				loader.runLoaderCommand().until(loader::isLoaded)
					
				)
					.until(loader::isLoaded)
					.andThen(stopShooterComponents())
			);
	}

	public Command autoIntake() {
		return launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelCommandGroup(	
				intake.intakeWheelCommand(),	
				simranIntakeAssist.downAndSpin().withTimeout(1),
				loader.runLoaderCommand().until(loader::isLoaded)
					
				)
					.until(loader::isLoaded).withTimeout(3.5)
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

	public Command lobShot() {
		return launcherAngle.speakerAngleCommand()
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						loader.feedShooterCommand().until(loader::isNotLoaded)
					),
				launcher.lobLauncherCommand()
				)
			)
			.andThen(stopShooterComponents());
	}

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

	public Command ampShotAssist() {
		return new ParallelDeadlineGroup(launcherAngle.ampAngleCommand(), new WaitCommand(0.75), ampAssist.up())
		
			.andThen(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						new WaitCommand(0.75),
						loader.feedShooterCommand().until(loader::isNotLoaded)
					),
					launcher.ampLauncherCommand()
				)
			)
			.andThen(new WaitCommand(0.75), ampAssist.down(), stopShooterComponents());
	};

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
