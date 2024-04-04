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
import frc.robot.subsystems.FisheyeCamera;
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
    public FisheyeCamera fisheye;
    public LedManager ledManager;
    public AmpAssist ampAssist;

  }

  private Subsystems subsystems;

  public class Commands {
    public Command swerveCmd;
    public Command swerveRobotRelativeCmd;
    public Command shooterFeed;
    public Command stopShooterComponents;
    public Command eject;
    public Command climberManual;
    public Command climberUp;
    public Command climberDown;
    public Command speakerAngle;
    public Command ampAngle;
    public Command speakerLauncher;
    public Command speakerFarLauncher;
    public Command ampLauncher;
    public Command zeroGyro;
    public Command sourceIntake;
    public Command groundIntake;
    public Command fullIntake;
    public Command ampAssistUp;
    public Command ampAssistDown;
    public Command ampLauncherAssist;
    public Command lobLauncher;
  }

  private Commands commands;
  private CommandCombinator combinator;

  public class StateSuppliers {
    public BooleanSupplier groundIntakeRunningAmpAngle;
    public BooleanSupplier readyToIntakeFromSource;
    public BooleanSupplier readyToShoot;
    public BooleanSupplier noteLoaded;
    public BooleanSupplier slowMode;
  }

  private StateSuppliers stateSuppliers;

  public RobotContainer() {
    /**
     * initalize controllers here
     */
    {
      this.controllers = new Controllers();
      this.controllers.driverController = new CommandXboxController(0);
      this.controllers.secondaryController = new CommandXboxController(1);
    }

    /**
     * initalize subsystems here
     */
    {
      this.subsystems = new Subsystems();
      // distanceSensorMXP = new DistanceSensorMXP();
      this.subsystems.ampAssist = new AmpAssist();
      this.subsystems.leds = new LedController_MultiAccess(new LedController());
      this.subsystems.ledManager = new LedManager(subsystems.leds.getController());
      this.subsystems.fisheye = new FisheyeCamera();
      this.subsystems.intake = new Intake();
      this.subsystems.simranIntakeAssist = new SimranIntakeAssist();
      this.subsystems.launcher = new LauncherV2();
      this.subsystems.launcherAngle = new DumbLauncherAngle();
      this.subsystems.distanceSensor = new DistanceSensor();
      this.subsystems.loader = new LoaderV2();
      this.subsystems.climber = new Climber();
      this.subsystems.swerve = new Swerve();

      // the death zone
    }

    /**
     * initalize commands here
     */
    {
      this.commands = new Commands();
      // combine subsystem commands into sequential/parallel command groups
      combinator = new CommandCombinator(this.subsystems);
      // drive swerve, slow mode with b
      this.commands.swerveCmd = new RunCommand(
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

      this.commands.swerveRobotRelativeCmd = new RunCommand(
        () -> {
          double y = MathUtil.applyDeadband(this.controllers.secondaryController.getLeftY(), 0.1);
          double x = MathUtil.applyDeadband(this.controllers.secondaryController.getLeftX(), 0.1);
          double w = MathUtil.applyDeadband(this.controllers.secondaryController.getRightX(), 0.1);
  
        Vector2D vector = new Vector2D(y, x, false);
            this.subsystems.swerve.drive(vector, -w, false);
  
        }, this.subsystems.swerve);

      // set gyro yaw to 0
      this.commands.zeroGyro = Pgyro.zeroGyroCommand();

      // manual climber operation, no limits
      this.commands.climberManual = this.subsystems.climber.runRaw(
          () -> {
            return this.controllers.secondaryController.getRightTriggerAxis()
                - this.controllers.secondaryController.getLeftTriggerAxis();
          });
      // move climber up with limits
      this.commands.climberUp = this.subsystems.climber.moveUpCommand();
      // move climber down with limits
      this.commands.climberDown = this.subsystems.climber.moveDownCommand();

      // backup angle to amp/speaker close shot
      this.commands.ampAngle = this.subsystems.launcherAngle.ampAngleCommand();
      // backup angle to intake/speaker far shot
      this.commands.speakerAngle = this.subsystems.launcherAngle.speakerAngleCommand();

      // backup shooter feed command
      this.commands.shooterFeed = this.subsystems.loader.feedShooterCommand()
          .until(this.subsystems.distanceSensor::isNotePresent).andThen(this.subsystems.loader.stopLoaderCommand());
      // stop all shooter components
      this.commands.stopShooterComponents = combinator.stopShooterComponents();

      // spin up launcher, shoot to speaker after 0.5 seconds
      this.commands.speakerLauncher = combinator.speakerShot();
      // spin up launcher, shoot to speaker from intake angle after 0.5 seconds
      this.commands.speakerFarLauncher = combinator.speakerFarShot();
      // spin up launcher, shoot to amp after 0.5 seconds
      this.commands.ampLauncher = combinator.ampShot();

      this.commands.lobLauncher = combinator.lobShot();

      this.commands.ampLauncherAssist = combinator.ampShotAssist();

      // intake note from source, auto stop
      this.commands.sourceIntake = combinator.sourceIntake();
      // intake note from ground, auto stop
      this.commands.groundIntake = combinator.groundIntake();
      // intake note from source and ground, auto stop
      this.commands.fullIntake = combinator.fullIntake();

      // amp assist up and down
      this.commands.ampAssistUp = this.subsystems.ampAssist.up();
      this.commands.ampAssistDown = this.subsystems.ampAssist.down();

      // eject note
      this.commands.eject = combinator.eject();

      // spin up launcher, shoot to speaker from intake angle after 0.5 seconds
      this.commands.speakerFarLauncher = combinator.speakerFarShot();
    }

    /**
     * initalize state suppliers here
     */
    {
      this.stateSuppliers = new StateSuppliers();
      this.stateSuppliers.groundIntakeRunningAmpAngle = () -> this.subsystems.intake.isRunning()
          && this.subsystems.launcherAngle.isUp();
      this.stateSuppliers.readyToIntakeFromSource = () -> this.subsystems.launcher.isRunningIntake()
          && !this.subsystems.distanceSensor.isNotePresent() && this.subsystems.launcherAngle.isUp();
      this.stateSuppliers.readyToShoot = () -> this.subsystems.distanceSensor.isNotePresent()
          && this.subsystems.swerve.atTargetAngle()
          && false; // disabled not ready
      this.stateSuppliers.noteLoaded = () -> this.subsystems.distanceSensor.isNotePresent();
      this.stateSuppliers.slowMode = () -> this.controllers.driverController.b().getAsBoolean();
    }

    /**
     * configure controller bindings here
     */

    {
      this.subsystems.ledManager
          .setDefaultCommand(this.subsystems.ledManager.LedControllingCommand(this.stateSuppliers));

      // add zero gyro button
      Shuffleboard.getTab("Settings").add("Zero Gyro", this.commands.zeroGyro);

      this.subsystems.swerve.setDefaultCommand(this.commands.swerveCmd); // both joysticks
      this.subsystems.climber.setDefaultCommand(this.commands.climberManual); // right trigger and left trigger

      this.controllers.driverController.x().onTrue(this.commands.zeroGyro);

      this.controllers.driverController.y().onTrue(this.commands.sourceIntake);
      this.controllers.driverController.a().onTrue(this.commands.groundIntake);

      this.controllers.driverController.leftTrigger().onTrue(this.commands.speakerAngle);
      this.controllers.driverController.rightTrigger().onTrue(this.commands.ampAngle);
      this.controllers.driverController.leftBumper().onTrue(this.commands.ampLauncherAssist);
      this.controllers.driverController.rightBumper().onTrue(this.commands.speakerLauncher);

      this.controllers.driverController.start().onTrue(this.commands.eject);

      this.controllers.driverController.povDown().onTrue(this.commands.shooterFeed);

      this.controllers.secondaryController.y().onTrue(this.commands.speakerLauncher);

      this.controllers.secondaryController.b().onTrue(this.commands.ampLauncherAssist);

      this.controllers.secondaryController.start().onTrue(this.commands.eject);
      this.controllers.secondaryController.x().onTrue(this.commands.stopShooterComponents);
      this.controllers.secondaryController.a().onTrue(this.commands.speakerFarLauncher);

      this.controllers.secondaryController.rightBumper().whileTrue(this.commands.climberDown);
      this.controllers.secondaryController.leftBumper().whileTrue(this.commands.climberUp);

      this.controllers.secondaryController.povDown().onTrue(this.commands.fullIntake);
      this.controllers.secondaryController.povUp().onTrue(this.subsystems.simranIntakeAssist.upAndStop());

      this.controllers.secondaryController.povLeft().onTrue(this.commands.groundIntake);
    }
    // named command init
    {
      NamedCommands.registerCommand("intake", combinator.autoIntake());
      NamedCommands.registerCommand("closeShoot", this.commands.speakerLauncher);
      NamedCommands.registerCommand("farShoot", this.commands.speakerFarLauncher);
      NamedCommands.registerCommand("stop", this.commands.stopShooterComponents);
    }

    // auton config
    {
      autoChooser = new SendableChooser<>();
      autoChooser.addOption("disrupt", AutoBuilder.buildAuto("disruptor"));
      autoChooser.addOption("playoff auto", AutoBuilder.buildAuto("playoff auto"));
      autoChooser.addOption("Shoot Only, Any Pos", AutoBuilder.buildAuto("Shoot Only, Any Pos"));
      autoChooser.addOption("Middle 4 Note - WEEK 5", AutoBuilder.buildAuto("Middle 4 Note - WEEK 5"));
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
      // autoChooser.addOption("simranintaketestjustintake", combinator.fullIntake());
      SmartDashboard.putData("Auton chooser", autoChooser);
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void disabledPeriodic() {
    this.subsystems.swerve.disabledPeriodic();

  }
}
