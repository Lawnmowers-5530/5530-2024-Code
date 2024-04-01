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

	
	//Eject Ring
	public Command eject() {
		return new ParallelCommandGroup(
				intake.ejectCommand(),
				loader.ejectCommand(),
				simranIntakeAssist.downAndEject()
				);
	}


	//Stop all Componets, and retract everything
	public Command stopShooterComponents() {
		return new ParallelCommandGroup(
				
				intake.stopIntakeWheelCommand(),
				loader.stopLoaderCommand(),
				launcher.stopLauncherCommand(), 
				simranIntakeAssist.upAndStop(),
				ampAssist.down()
				);
	}

	//Distance Sensor Fall back, stops all componets and brings ring back into gate wheels so it is no longer touching fly wheels
	public Command manualIntakeStop(){
		return new SequentialCommandGroup(
			stopShooterComponents(),
			loader.ejectCommand(),
			new WaitCommand(0.25), //subject to change
			stopShooterComponents()
		);
	}

	//Intake Fall back, intakes from source above
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

	//Intakes from ground using ONLY internal
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

	
	//Intakes from ground using external and internal
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

	//FOR AUTON, Intakes from ground using external and internal, but times out
	public Command autonIntake() {
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

	//FOR AUTON, feeds the already spun up shooter and turns of the componets
	public Command feedAndOff(){
		return new SequentialCommandGroup(
			loader.feedShooterCommand().until(loader::isNotLoaded).withTimeout(0.1),//TIME subject to change
			stopShooterComponents()
		);
	}

	public Command spinAndAngle(){
		return new ParallelCommandGroup(
		launcherAngle.ampAngleCommand(),
		launcher.speakerLauncherCommand()
		);
	}

	
	//Shoots into the speaker, from the close angle and high rpm
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

	
	//Shoots into the speaker, from the far angle and high rpm
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

	//Shoots from the centerline to into wing USED during auto - CURRENTLY ILLEGAL NEEDS TO CHANGE
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

	//Shoots into amp WITHOUT amp assist arm
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

	
	//shoots into amp WITH amp assist arm
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

	
}
