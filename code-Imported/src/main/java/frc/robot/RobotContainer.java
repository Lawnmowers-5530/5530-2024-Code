// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import frc.lib.ShotCalculator;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.commands.LauncherIntake;
import frc.robot.commands.VelocityLauncher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DashboardIndicators;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;
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

  private DashboardIndicators dash;

  private CommandXboxController driverController;
  private CommandXboxController secondaryController;

  private Command intakeCommand;
  private Command swerveCmd;
  private Command resetGyro;
  private Command shooterFeed;
  private Command stopShooterComponents;
  private Command eject;

  private Command climberCommandManual;
  private Command climberUp;
  private Command climberDown;

  private Command speakerAngle;
  private Command ampAngle;
  private Command speakerShot;
  private Command ampShot;

  private Command pathFindCommand;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    secondaryController = new CommandXboxController(1);

    createSubsystems();

    createCommands();

    NamedCommands.registerCommand("intake", intakeCommand);
    NamedCommands.registerCommand("shoot", ampShot);
    NamedCommands.registerCommand("feed", shooterFeed);
    NamedCommands.registerCommand("stop", stopShooterComponents);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auton chooser", autoChooser);
  }

  private void createSubsystems() {
    dash = new DashboardIndicators(distanceSensor);
    intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);
    launcher = new LauncherV2();
    launcherAngle = new DumbLauncherAngle(
        Constants.LauncherAngleConstants.motorPort,
        Constants.LauncherAngleConstants.isReversed);
    distanceSensor = new DistanceSensor();
    loader = new LoaderV2(
        Constants.LoaderConstants.leftMotorPort,
        Constants.LoaderConstants.rightMotorPort,
        Constants.LoaderConstants.isReversed, distanceSensor);
    climber = new Climber();

    swerve = new Swerve();

    // the death zone
  }

  private void createCommands() {
    pathFindCommand = AutoBuilder.pathfindToPose(
        ShotCalculator.getDistPose(swerve.getPose()), Constants.PathPlannerConstants.constraints);

    climberCommandManual = new RunCommand(
        () -> {
          climber.run(-secondaryController.getLeftTriggerAxis() + secondaryController.getRightTriggerAxis());
        }, climber);

    climberUp = new RunCommand(
        () -> {
          climber.moveDown();
        }, climber);

    climberDown = new RunCommand(
        () -> {
          climber.moveUp();
        }, climber);

    ampAngle = new RunCommand(
        () -> {
          launcherAngle.forceUp();
        }, launcherAngle);

    speakerAngle = new RunCommand(
        () -> {
          launcherAngle.forceDown();
        }, launcherAngle);

    ampShot = new VelocityLauncher(
        launcher,
        () -> {
          return Constants.LauncherConstants.LAUNCHER_LOW_REVS;
        },
        () -> {
          return Constants.LauncherConstants.LAUNCHER_LOW_REVS
              / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
        });

    speakerShot = new VelocityLauncher(
        launcher,
        () -> {
          return Constants.LauncherConstants.LAUNCHER_HIGH_REVS;
        },
        () -> {
          return Constants.LauncherConstants.LAUNCHER_HIGH_REVS
              / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
        });

    intakeCommand = new RunCommand(
        () -> {
          loader.runUntilBeamBreak(Constants.LoaderConstants.loaderSpeed,
              Constants.LoaderConstants.loaderCutoffDistance,
              intake);
        }, new Subsystem[] { intake, loader });

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

    resetGyro = new RunCommand(
        () -> {
          Pgyro.zeroGyro();
        }, new Subsystem[] {});

    shooterFeed = new RunCommand(
        () -> {
          loader.run(0.2);
        }, loader);

    stopShooterComponents = new RunCommand(
        () -> {
          loader.run(0);
          launcher.setSpeed(0, 0);
          intake.run(0);
        }, new Subsystem[] { loader, launcher });

    eject = new RunCommand(
        () -> {
          loader.run(-0.3);
          intake.run(-0.3);
        }, new Subsystem[] { loader, intake });
  }

  private void configureBindings() {
    dash.isLoaded();

    swerve.setDefaultCommand(swerveCmd);
    climber.setDefaultCommand(climberCommandManual);

    driverController.x().whileTrue(resetGyro); // if not working use repeatcommand
    driverController.y().onTrue(new LauncherIntake(distanceSensor, loader, launcher,
        Constants.LauncherIntakeConstants.theshold, Constants.LauncherIntakeConstants.speed));
    driverController.a().onTrue(intakeCommand);
    driverController.leftTrigger().onTrue(ampAngle);
    driverController.rightTrigger().onTrue(speakerAngle);
    driverController.leftBumper().onTrue(ampShot);
    driverController.rightBumper().onTrue(speakerShot);
    driverController.start().onTrue(pathFindCommand);

    secondaryController.a().onTrue(ampShot);
    secondaryController.y().onTrue(speakerShot);
    secondaryController.start().onTrue(eject);
    secondaryController.leftBumper().onTrue(shooterFeed);
    secondaryController.rightBumper().onTrue(stopShooterComponents);
    secondaryController.leftBumper().whileTrue(climberUp);
    secondaryController.rightBumper().whileTrue(climberDown);
    secondaryController.povDown().onTrue(ampAngle);
    secondaryController.povUp().onTrue(speakerAngle);
    secondaryController.povLeft().onTrue(intakeCommand);
    secondaryController.povRight().onTrue(new LauncherIntake(distanceSensor, loader, launcher,
        Constants.LauncherIntakeConstants.theshold, Constants.LauncherIntakeConstants.speed));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
