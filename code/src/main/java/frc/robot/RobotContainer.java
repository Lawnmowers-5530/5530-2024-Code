// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  public class Controllers {
    public CommandXboxController driverController;
    public CommandXboxController secondaryController;
  }
  private Controllers controllers;
  public class Subsystems {
    public Swerve swerve;
    public DistanceSensor distanceSensor;
    public DumbLauncherAngle launcherAngle;
    public LoaderV2 loader;
    public LauncherV2 launcher;
    public Climber climber;
    public Intake intake;
    public SimranIntakeAssist simranIntakeAssist;
    public LedController_MultiAccess leds;
    public Camera fisheye;
    public LedManager ledManager;
    public AmpAssist ampAssist;

  }
  private Subsystems subsystems;

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
  private Command fullIntake;

  private Command ampAssistUp;
  private Command ampAssistDown;
  private Command ampLauncherAssist;

 
  
  private CommandCombinator combinator;

  private BooleanSupplier groundIntakeRunningAmpAngle;
  private BooleanSupplier readyToIntakeFromSource;
  private BooleanSupplier readyToShoot;
  private BooleanSupplier noteLoaded;
  private BooleanSupplier slowMode;

  public RobotContainer() {
    this.controllers.driverController = new CommandXboxController(0);
    this.controllers.secondaryController = new CommandXboxController(1);

    createSubsystems();

    createCommands();

    NamedCommands.registerCommand("intake", combinator.autoIntake());
    //NamedCommands.registerCommand("intake", new InstantCommand ( () -> {CommandScheduler.getInstance().schedule(combinator.autoIntake());}));
    NamedCommands.registerCommand("closeShoot", speakerLauncher);
    NamedCommands.registerCommand("farShoot", speakerFarLauncher);
    NamedCommands.registerCommand("stop", stopShooterComponents);
    

    createStateSuppliers();

    configureBindings();
    //auton config blocks
    {
      autoChooser = new SendableChooser<>();
      autoChooser.addOption("Shoot Only, Any Pos", AutoBuilder.buildAuto("Shoot Only, Any Pos"));
      autoChooser.addOption("Parallel Middle Auto - WEEK 5", AutoBuilder.buildAuto("Parallel Middle Auto - WEEK 5"));
      autoChooser.addOption("Shoot and Leave Amp - WEEK 5", AutoBuilder.buildAuto("Shoot and Leave Amp - WEEK 5"));
      autoChooser.addOption("Amp 3 Note - WEEK 5", AutoBuilder.buildAuto("Amp 3 Note - WEEK 5"));
      autoChooser.addOption("Shoot and Leave Middle - WEEK 5", AutoBuilder.buildAuto("Shoot and Leave Middle - WEEK 5"));
      autoChooser.addOption("Shoot and Leave Source - WEEK 5", AutoBuilder.buildAuto("Shoot and Leave Source - WEEK 5"));
      // autoChooser.addOption("simranintaketestjustintake", combinator.fullIntake());
      SmartDashboard.putData("Auton chooser", autoChooser);
    }
  }

  private void createSubsystems() {
    //distanceSensorMXP = new DistanceSensorMXP();
    this.subsystems.ampAssist = new AmpAssist();
    this.subsystems.leds = new LedController_MultiAccess(new LedController(0, StripType.Adressable, "Competition"));
    this.subsystems.ledManager = new LedManager(subsystems.leds.getController());
    this.subsystems.fisheye = new Camera("fisheye", 0, 320, 240, 300);
    this.subsystems.intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);
    this.subsystems.simranIntakeAssist = new SimranIntakeAssist( Constants.ExternalIntakeConstants.pivotMotorPort, 
      Constants.ExternalIntakeConstants.rollerMotorPort,
      Constants.ExternalIntakeConstants.isReversed);
    this.subsystems.launcher = new LauncherV2();
    this.subsystems.launcherAngle = new DumbLauncherAngle(Constants.LauncherAngleConstants.motorPort, Constants.LauncherAngleConstants.isReversed);
    this.subsystems.distanceSensor = new DistanceSensor();
    this.subsystems.loader = new LoaderV2(Constants.LoaderConstants.leftMotorPort, Constants.LoaderConstants.rightMotorPort, Constants.LoaderConstants.isReversed, subsystems.distanceSensor);
    this.subsystems.climber = new Climber();
    this.subsystems.swerve = new Swerve();

    // the death zone
  }

  private void createCommands() {
    // combine subsystem commands into sequential/parallel command groups
    combinator = new CommandCombinator(this.subsystems);

    // drive swerve, slow mode with b
    swerveCmd = new RunCommand(
        () -> {
          double y = MathUtil.applyDeadband(this.controllers.driverController.getLeftY(), 0.06);
          double x = MathUtil.applyDeadband(this.controllers.driverController.getLeftX(), 0.06);
          double w = MathUtil.applyDeadband(this.controllers.driverController.getRightX(), 0.06);

          Vector2D vector = new Vector2D(y, x, false);
          this.subsystems.swerve.drive(vector, -w, true);

          if (this.controllers.driverController.b().getAsBoolean()) {
            this.subsystems.swerve.drive(VectorOperator.scalarMultiply(vector, 0.5), -w / 2, true);
          }

        }, this.subsystems.swerve);

    // set gyro yaw to 0
    zeroGyro = Pgyro.zeroGyroCommand();

    // manual climber operation, no limits
    climberManual = this.subsystems.climber.runRaw(
      () -> {
        return this.controllers.secondaryController.getRightTriggerAxis() - this.controllers.secondaryController.getLeftTriggerAxis();
      }
    );
    // move climber up with limits
    climberUp = this.subsystems.climber.moveUpCommand();
    // move climber down with limits
    climberDown = this.subsystems.climber.moveDownCommand();

    // backup angle to amp/speaker close shot
    ampAngle = this.subsystems.launcherAngle.ampAngleCommand();
    // backup angle to intake/speaker far shot
    speakerAngle = this.subsystems.launcherAngle.speakerAngleCommand();

    // backup shooter feed command
    shooterFeed = this.subsystems.loader.feedShooterCommand().until(this.subsystems.loader::isNotLoaded).andThen(this.subsystems.loader.stopLoaderCommand());
    // stop all shooter components
    stopShooterComponents = combinator.stopShooterComponents();

    // spin up launcher, shoot to speaker after 0.5 seconds
    speakerLauncher = combinator.speakerShot();
    // spin up launcher, shoot to speaker from intake angle after 0.5 seconds
    speakerFarLauncher = combinator.speakerFarShot();
    // spin up launcher, shoot to amp after 0.5 seconds
    ampLauncher = combinator.ampShot();

    ampLauncherAssist = combinator.ampShotAssist();

    // intake note from source, auto stop
    sourceIntake = combinator.sourceIntake();
    // intake note from ground, auto stop
    groundIntake = combinator.groundIntake();
    fullIntake = combinator.fullIntake();

    //amp assist up
    ampAssistUp = this.subsystems.ampAssist.up();
    ampAssistDown = this.subsystems.ampAssist.down();

    // eject note
    // make eject a toggle button
    eject = combinator.eject();

    speakerFarLauncher = combinator.speakerFarShot();
  }

  private void createStateSuppliers() {
    groundIntakeRunningAmpAngle = () -> this.subsystems.intake.isRunning() && this.subsystems.launcherAngle.isUp();
    readyToIntakeFromSource = () -> this.subsystems.launcher.isRunningIntake() && !this.subsystems.loader.isLoaded() && this.subsystems.launcherAngle.isUp();
    readyToShoot = () -> this.subsystems.loader.isLoaded() && this.subsystems.swerve.atTargetAngle() && false; //disabled not ready
    noteLoaded = () -> this.subsystems.loader.isLoaded();
    slowMode = () -> this.controllers.driverController.b().getAsBoolean();
  }

  private void configureBindings() {
    this.subsystems.ledManager.setDefaultCommand(this.subsystems.ledManager.LedControllingCommand(
        groundIntakeRunningAmpAngle,
        readyToIntakeFromSource,
        readyToShoot,
        noteLoaded,
        slowMode));
    //add zero gyro button
    Shuffleboard.getTab("Settings").add("Zero Gyro", zeroGyro);

    this.subsystems.swerve.setDefaultCommand(swerveCmd); // both joysticks
    this.subsystems.climber.setDefaultCommand(climberManual); // right trigger and left trigger

    this.controllers.driverController.x().onTrue(zeroGyro);

    this.controllers.driverController.y().onTrue(sourceIntake);
    this.controllers.driverController.a().onTrue(groundIntake);

    this.controllers.driverController.leftTrigger().onTrue(speakerAngle);
    this.controllers.driverController.rightTrigger().onTrue(ampAngle);
    this.controllers.driverController.leftBumper().onTrue(ampLauncherAssist);
    this.controllers.driverController.rightBumper().onTrue(speakerLauncher);

    this.controllers.driverController.start().onTrue(eject);

    this.controllers.driverController.povDown().onTrue(shooterFeed);

    this.controllers.secondaryController.y().onTrue(speakerLauncher);

    this.controllers.secondaryController.b().onTrue(ampLauncherAssist);

    this.controllers.secondaryController.start().onTrue(eject);
    this.controllers.secondaryController.x().onTrue(stopShooterComponents);
    this.controllers.secondaryController.a().onTrue(speakerFarLauncher);

    this.controllers.secondaryController.rightBumper().whileTrue(climberDown);
    this.controllers.secondaryController.leftBumper().whileTrue(climberUp);

    //secondaryController.povDown().onTrue(ampAngle);
    //secondaryController.povUp().onTrue(speakerAngle);

    this.controllers.secondaryController.povDown().onTrue(fullIntake);
    this.controllers.secondaryController.povUp().onTrue(this.subsystems.simranIntakeAssist.upAndStop());

    this.controllers.secondaryController.povLeft().onTrue(groundIntake);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void disabledPeriodic() {
    this.subsystems.swerve.disabledPeriodic();

  }
}
