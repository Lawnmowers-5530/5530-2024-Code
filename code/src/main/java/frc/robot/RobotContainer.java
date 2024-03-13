// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.commands.CommandCombinator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController_MultiAccess;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LedController.fixedPalattePatternType;
import frc.robot.subsystems.LedController.stripType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer implements Loggable {
  private SendableChooser<Command> autoChooser;

  private Swerve swerve;
  private DistanceSensor distanceSensor;
  private DumbLauncherAngle launcherAngle;
  private LoaderV2 loader;
  private LauncherV2 launcher;
  private Climber climber;
  private Intake intake;
  private LedController_MultiAccess leds;

  private CommandXboxController driverController;
  private CommandXboxController secondaryController;

  private Command swerveCmd;
  private Command shooterFeed;
  private Command stopShooterComponents;
  private Command eject;

  private Command climberManual;
  private Command climberUp;
  private Command climberDown;

  private Command speakerAngle;
  private Command ampAngle;
  private Command speakerLauncher;
  private Command speakerFarLauncher;
  private Command ampLauncher;
  private Command zeroGyro;
  private Command sourceIntake;
  private Command groundIntake;

  private CommandCombinator combinator;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    secondaryController = new CommandXboxController(1);

    createSubsystems();

    createCommands();

    NamedCommands.registerCommand("intake", groundIntake);
    NamedCommands.registerCommand("shoot", speakerFarLauncher);
    NamedCommands.registerCommand("stop", stopShooterComponents);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton chooser", autoChooser);
  }

  private void createSubsystems() {
    leds = new LedController_MultiAccess(new LedController(0, stripType.Adressable));
    intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);
    launcher = new LauncherV2();
    launcherAngle = new DumbLauncherAngle(
        Constants.LauncherAngleConstants.motorPort,
        Constants.LauncherAngleConstants.isReversed);
    distanceSensor = new DistanceSensor(leds.getController());
    loader = new LoaderV2(
        Constants.LoaderConstants.leftMotorPort,
        Constants.LoaderConstants.rightMotorPort,
        Constants.LoaderConstants.isReversed, distanceSensor);
    climber = new Climber();

    swerve = new Swerve();

    // the death zone
  }

  private void createCommands() {
    //combine subsystem commands into sequential/parallel command groups
    combinator = new CommandCombinator(climber, intake, launcher, loader, launcherAngle, distanceSensor);

    //drive swerve, slow mode with b
    swerveCmd = new RunCommand(
        () -> {
          double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
          double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
          double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);

          Vector2D vector = new Vector2D(y, x, false);
          swerve.drive(vector, -w, true);

          if (driverController.b().getAsBoolean()) {
            swerve.drive(VectorOperator.scalarMultiply(vector, 0.5), -w / 2, true);
          }

        }, swerve);

    //set gyro yaw to 0
    zeroGyro = Pgyro.zeroGyroCommand();

    //manual climber operation, no limits
    climberManual = climber.runRaw(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis());
    //move climber up with limits
    climberUp = climber.moveUpCommand();
    //move climber down with limits
    climberDown = climber.moveDownCommand();

    //backup angle to amp/speaker close shot
    ampAngle = launcherAngle.ampAngleCommand();
    //backup angle to intake/speaker far shot
    speakerAngle = launcherAngle.speakerAngleCommand();

    //backup shooter feed command
    shooterFeed = loader.feedShooterCommand().until(loader::isNotLoaded).andThen(loader.stopLoaderCommand());
    //stop all shooter components
    stopShooterComponents = combinator.stopShooterComponents();

    //spin up launcher, shoot to speaker after 0.5 seconds
    speakerLauncher = combinator.speakerShot();
    //spin up launcher, shoot to speaker from intake angle after 0.5 seconds
    speakerFarLauncher = combinator.speakerFarShot();
    //spin up launcher, shoot to amp after 0.5 seconds
    ampLauncher = combinator.ampShot();

    //intake note from source, auto stop
    sourceIntake = combinator.sourceIntake();
    //intake note from ground, auto stop
    groundIntake = combinator.groundIntake();

    //eject note
    //make eject a toggle button
    eject = combinator.eject();
  }

  private void configureBindings() {

    swerve.setDefaultCommand(swerveCmd); // both joysticks
    climber.setDefaultCommand(climberManual); // right trigger and left trigger

    driverController.b().onTrue(leds.LedControllerCommand(fixedPalattePatternType.Rainbow, 2));

    driverController.x().onTrue(zeroGyro);

    driverController.y().onTrue(sourceIntake);
    driverController.a().onTrue(groundIntake);

    driverController.leftTrigger().onTrue(speakerAngle);
    driverController.rightTrigger().onTrue(ampAngle);
    driverController.leftBumper().onTrue(ampLauncher);
    driverController.rightBumper().onTrue(speakerLauncher);

    driverController.start().onTrue(eject);

    driverController.povDown().onTrue(shooterFeed);

    secondaryController.y().onTrue(speakerLauncher);

    secondaryController.b().onTrue(ampLauncher);

    secondaryController.start().onTrue(eject);
    secondaryController.a().onTrue(stopShooterComponents);

    secondaryController.leftBumper().whileTrue(climberDown);
    secondaryController.rightBumper().whileTrue(climberUp);

    secondaryController.povDown().onTrue(ampAngle);
    secondaryController.povUp().onTrue(speakerAngle);

    secondaryController.povLeft().onTrue(groundIntake);
    secondaryController.povRight().onTrue(sourceIntake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
