// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import frc.lib.ShotCalculator;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.commands.ClimberLimited;
import frc.robot.commands.ClimberManual;
import frc.robot.commands.IntakeOffFloor;
import frc.robot.commands.AngleLauncher;
import frc.robot.commands.LauncherIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunLauncher;
import frc.robot.commands.RunLoader;
import frc.robot.commands.StopCommand;
import frc.robot.commands.VelocityLauncher;
import frc.robot.commands.AngleLauncher.Angle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DashboardIndicators;
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
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

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

  private Command highAngle;
  private Command lowAngle;
  private Command speakerShot;
  private Command ampShot;

  private Command launcherIntakeCommand;

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
    leds = new LedController_MultiAccess(new LedController(0, stripType.Adressable));
    intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);
    launcher = new LauncherV2();
    launcherAngle = new DumbLauncherAngle(
        Constants.LauncherAngleConstants.motorPort,
        Constants.LauncherAngleConstants.isReversed);
    distanceSensor = new DistanceSensor(leds.getController());
    dash = new DashboardIndicators(distanceSensor);
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

    climberCommandManual = new ClimberManual(climber, () -> {return secondaryController.getRightTriggerAxis();});

    climberUp = new ClimberLimited(climber, Constants.ClimberConstants.speed);

    climberDown = new ClimberLimited(climber, -Constants.ClimberConstants.speed);

    lowAngle = new AngleLauncher(launcherAngle, Angle.UP);

    highAngle = new AngleLauncher(launcherAngle, Angle.DOWN);

    intakeCommand = new IntakeOffFloor(intake, loader, 0, 0);

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

    resetGyro = new InstantCommand(() -> {Pgyro.zeroGyro();});

    shooterFeed = new RunLoader(loader, 0.2); //TODO: make this a constant

    StopCommand<LoaderV2> stopLoader = new StopCommand<>(loader);
    StopCommand<Intake> stopIntake = new StopCommand<>(intake);
    StopCommand<LauncherV2> stopLauncher = new StopCommand<>(launcher);
    stopShooterComponents = new ParallelCommandGroup(new Command[] { stopLoader, stopIntake, stopLauncher });
    

    RunLoader ejectLoader = new RunLoader(loader, -0.3);
    RunIntake ejectIntake = new RunIntake(intake, -0.3);
    eject = new ParallelCommandGroup(new Command[] { ejectLoader, ejectIntake });

    launcherIntakeCommand = new LauncherIntake(distanceSensor, loader, launcher,
    Constants.LauncherIntakeConstants.theshold, Constants.LauncherIntakeConstants.speed);


    ampShot = new RunLauncher(launcher, Constants.LauncherConstants.LAUNCHER_LOW_REVS, Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);

    speakerShot = new RunLauncher(launcher, Constants.LauncherConstants.LAUNCHER_HIGH_REVS, Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
  }
  private void configureBindings() {
    dash.isLoaded();

    swerve.setDefaultCommand(swerveCmd);
    climber.setDefaultCommand(climberCommandManual);

    driverController.b().onTrue(new RunCommand(
        () -> {
          LedControllerProxy ledProxy = leds.getController();
          ledProxy.setPattern(fixedPalattePatternType.ColorWavesOcean, 1);
        }, new Subsystem[] {}));

    driverController.x().whileTrue(resetGyro); // if not working use repeatcommand
    
    SequentialCommandGroup launcherIntakeGroup = new SequentialCommandGroup(new SequentialCommandGroup(new Command[] {highAngle, launcherIntakeCommand}));
    driverController.y().onTrue(launcherIntakeGroup);
    
    
    SequentialCommandGroup floorIntakeGroup = new SequentialCommandGroup(new SequentialCommandGroup(new Command[] {highAngle, intakeCommand}));
    driverController.a().onTrue(floorIntakeGroup);

    driverController.leftTrigger().onTrue(highAngle);
    driverController.rightTrigger().onTrue(lowAngle);
    
    SequentialCommandGroup ampShotGroup = new SequentialCommandGroup(new SequentialCommandGroup(new Command[] {lowAngle, ampShot}));
    driverController.leftBumper().onTrue(ampShotGroup);

    SequentialCommandGroup speakerShotGroup = new SequentialCommandGroup(new SequentialCommandGroup(new Command[] {lowAngle, speakerShot}));
    driverController.rightBumper().onTrue(speakerShotGroup);

    driverController.start().onTrue(eject);
    driverController.povDown().onTrue(shooterFeed);

    // secondaryController.a().onTrue(ampShot);
    secondaryController.y().onTrue(speakerShotGroup);
      
    secondaryController.b().onTrue(ampShotGroup);
    secondaryController.start().onTrue(shooterFeed);
    secondaryController.a().onTrue(stopShooterComponents);
    secondaryController.rightBumper().onTrue(stopShooterComponents);
    secondaryController.leftBumper().whileTrue(climberUp);
    secondaryController.rightBumper().whileTrue(climberDown);
    secondaryController.povDown().onTrue(lowAngle);
    secondaryController.povUp().onTrue(highAngle);
    secondaryController.povLeft().onTrue(intakeCommand);
    secondaryController.povRight().onTrue(launcherIntakeGroup);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
