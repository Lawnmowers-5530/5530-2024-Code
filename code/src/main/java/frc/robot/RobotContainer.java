// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.commands.CommandCombinator;
import frc.robot.subsystems.AmpAssist;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DistanceSensorMXP;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LedController;
import frc.robot.subsystems.LedController_MultiAccess;
import frc.robot.subsystems.LedManager;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.SimranIntakeAssist;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LedController.StripType;

import java.util.function.BooleanSupplier;

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
  private SimranIntakeAssist simranIntakeAssist;
  private LedController_MultiAccess leds;
  private Camera fisheye;
  private LedManager ledManager;
  private AmpAssist ampAssist;
  private DistanceSensorMXP distanceSensorMXP;
  private CommandXboxController driverController;
  private CommandXboxController secondaryController;

  private Command swerveCmd;
  private Command robotRelativeCmd;
  private Command shooterFeed;
  private Command eject;

  private Command climberManual;

  private Command zeroGyro;


  

  private CommandCombinator combinator;

  private BooleanSupplier groundIntakeRunningAmpAngle;
  private BooleanSupplier readyToIntakeFromSource;
  private BooleanSupplier readyToShoot;
  private BooleanSupplier noteLoaded;
  private BooleanSupplier slowMode;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    secondaryController = new CommandXboxController(1);

    createSubsystems();

    createCommands();

    NamedCommands.registerCommand("intake", combinator.autonIntake());
    NamedCommands.registerCommand("spinAndAngle", combinator.spinAndAngle());
    NamedCommands.registerCommand("feedAndOff", combinator.feedAndOff());
    NamedCommands.registerCommand("closeShoot", combinator.speakerShot());
    NamedCommands.registerCommand("farShoot", combinator.speakerFarShot());
    NamedCommands.registerCommand("stop", combinator.stopShooterComponents());
    NamedCommands.registerCommand("lob", combinator.lobShot());

    createStateSuppliers();

    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("disrupt", AutoBuilder.buildAuto("disruptor"));
    autoChooser.addOption("playoff auto", AutoBuilder.buildAuto("playoff auto"));
    autoChooser.addOption("Shoot Only, Any Pos", AutoBuilder.buildAuto("Shoot Only, Any Pos"));
    
    autoChooser.addOption("---", new InstantCommand());
    autoChooser.addOption("Sped Up Middle 4 Note - WEEK 5", AutoBuilder.buildAuto("Sped Up Middle 4 Note - WEEK 5"));
    autoChooser.addOption("Shoot In Path Middle 4 Note - WEEK 5", AutoBuilder.buildAuto("Shoot In Path Middle 4 Note - WEEK 5"));
    
    autoChooser.addOption("----", new InstantCommand());
    autoChooser.addOption("Amp 3 Note - WEEK 5", AutoBuilder.buildAuto("Amp 3 Note - WEEK 5"));
    autoChooser.addOption("Source 3 Note - WEEK 5", AutoBuilder.buildAuto("Source 3 Note - WEEK 5"));
    
    autoChooser.addOption("-----", new InstantCommand());
    autoChooser.addOption("Amp 2 Note - WEEK 5", AutoBuilder.buildAuto("Amp 2 Note - WEEK 5"));
    autoChooser.addOption("Source 2 Note - WEEK 5", AutoBuilder.buildAuto("Source 2 Note - WEEK 5"));
    
    autoChooser.addOption("--------", new InstantCommand());
    autoChooser.addOption("Shoot and Leave Amp - WEEK 5", AutoBuilder.buildAuto("Shoot and Leave Amp - WEEK 5"));
    autoChooser.addOption("Shoot and Leave Middle - WEEK 5", AutoBuilder.buildAuto("Shoot and Leave Middle - WEEK 5"));
    autoChooser.addOption("Shoot and Leave Source - WEEK 5", AutoBuilder.buildAuto("Shoot and Leave Source - WEEK 5"));
    SmartDashboard.putData("Auton chooser", autoChooser);
  }

  private void createSubsystems() {
    // distanceSensorMXP = new DistanceSensorMXP();
    ampAssist = new AmpAssist();
    leds = new LedController_MultiAccess(new LedController(0, StripType.Adressable, "Competition"));
    ledManager = new LedManager(leds.getController());
    fisheye = new Camera("fisheye", 0, 320, 240, 300);

    intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);
    simranIntakeAssist = new SimranIntakeAssist(Constants.ExternalIntakeConstants.pivotMotorPort,
        Constants.ExternalIntakeConstants.rollerMotorPort,
        Constants.ExternalIntakeConstants.isReversed);
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
    // combine subsystem commands into sequential/parallel command groups
    combinator = new CommandCombinator(climber, intake, launcher, loader, launcherAngle, distanceSensor, ampAssist,
        simranIntakeAssist);

    // drive swerve, slow mode with b
    swerveCmd = new RunCommand(
        () -> {
          double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.06);
          double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.06);
          double w = MathUtil.applyDeadband(driverController.getRightX(), 0.06);

          Vector2D vector = new Vector2D(y, x, false);
          swerve.drive(vector, -w, true);

          if (driverController.b().getAsBoolean()) {
            swerve.drive(VectorOperator.scalarMultiply(vector, 0.5), -w / 2, true);
          }

        }, swerve);

    robotRelativeCmd = new RunCommand(
      () -> {
        double y = MathUtil.applyDeadband(secondaryController.getLeftY(), 0.06);
        double x = MathUtil.applyDeadband(secondaryController.getLeftX(), 0.06);
        double w = MathUtil.applyDeadband(secondaryController.getRightX(), 0.06);

      Vector2D vector = new Vector2D(y, x, false);
          swerve.drive(vector, -w, false);

      }, swerve);

    // set gyro yaw to 0
    zeroGyro = Pgyro.zeroGyroCommand();

    // manual climber operation, no limits
    climberManual = climber.runRaw(() -> {
      return secondaryController.getRightTriggerAxis() - secondaryController.getLeftTriggerAxis();
    });
    
    // backup shooter feed command
    shooterFeed = loader.feedShooterCommand().until(loader::isNotLoaded).andThen(loader.stopLoaderCommand());
  
  }

  private void createStateSuppliers() {
    groundIntakeRunningAmpAngle = () -> intake.isRunning() && launcherAngle.isUp();
    readyToIntakeFromSource = () -> launcher.isRunningIntake() && !loader.isLoaded() && launcherAngle.isUp();
    readyToShoot = () -> loader.isLoaded() && swerve.atTargetAngle() && false; // disabled not ready
    noteLoaded = () -> loader.isLoaded();
    slowMode = () -> driverController.b().getAsBoolean();
  }

  private void configureBindings() {
    ledManager.setDefaultCommand(ledManager.LedControllingCommand(
        groundIntakeRunningAmpAngle,
        readyToIntakeFromSource,
        readyToShoot,
        noteLoaded,
        slowMode));
    // add zero gyro button
    Shuffleboard.getTab("Settings").add("Zero Gyro", zeroGyro);

    swerve.setDefaultCommand(swerveCmd); // both joysticks
    climber.setDefaultCommand(climberManual); // right trigger and left trigger

    driverController.x().onTrue(zeroGyro);

    driverController.y().onTrue(combinator.sourceIntake());
    driverController.a().onTrue(combinator.groundIntake());

    driverController.leftTrigger().onTrue(launcherAngle.speakerAngleCommand());
    driverController.rightTrigger().onTrue(launcherAngle.ampAngleCommand());
    driverController.leftBumper().onTrue(combinator.ampShotAssist());
    driverController.rightBumper().onTrue(combinator.speakerShot());

    driverController.start().onTrue(eject);

    driverController.povDown().onTrue(shooterFeed);

    secondaryController.y().onTrue(combinator.speakerShot());

    secondaryController.b().onTrue(combinator.ampShotAssist());

    secondaryController.start().onTrue(combinator.eject());
    secondaryController.x().onTrue(combinator.stopShooterComponents());
    secondaryController.a().onTrue(combinator.speakerFarShot());

    secondaryController.rightBumper().whileTrue(climber.moveDownCommand());
    secondaryController.leftBumper().whileTrue(climber.moveUpCommand());


    secondaryController.povDown().onTrue(combinator.fullIntake());
    secondaryController.povUp().onTrue(simranIntakeAssist.upAndStop());

    secondaryController.povLeft().onTrue(combinator.groundIntake());
    secondaryController.povRight().onTrue(combinator.manualIntakeStop());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void disabledPeriodic() {
    swerve.disabledPeriodic();

  }
}
