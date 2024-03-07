// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import frc.lib.ShotCalculator;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.commands.LauncherIntake;
import frc.robot.commands.VelocityLauncher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DumbLauncherAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherAngle;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.Loader;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer implements Loggable {
  private Field2d field;
  private SendableChooser<Command> autoChooser;
  private Swerve swerve;
  private DistanceSensor distanceSensor;
  private LauncherAngle launcherAngle;
  private LoaderV2 loader;
  private LauncherV2 launcher;

  private Climber climber;

  private DoubleSupplier shotAngleSupplier;

  private CommandXboxController driverController;

  private CommandXboxController secondaryController;

  private Intake intake;

  private Command intakeCommand;
  private Command swerveCmd;
  private Command resetGyro;
  private Command loadCommand;
  private Command testAngleCommand;
  private Command climberCommand;
  private Command shooterFeed;
  private Command stopShooterComponents;

  // Dumb launcher angle section

  private DumbLauncherAngle dumbLauncherAngle;
  private Command AngleDownCommand;
  private Command AngleUpCommand;
  private Command AngleRelaxCommand;

  // ----------------------------
  public RobotContainer() {
    driverController = new CommandXboxController(0);
    secondaryController = new CommandXboxController(1);

    createSubsystems();

    createCommands();

    NamedCommands.registerCommand("intake", intakeCommand);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auton chooser", autoChooser);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });
  }

  private void createCommands() {
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

        }, swerve);

    resetGyro = new RunCommand(
        () -> {
          Pgyro.zeroGyro();
        }, new Subsystem[] {});

    loadCommand = new RunCommand(
        () -> {
          loader.runUntilBeamBreak(Constants.LoaderConstants.loaderSpeed,
              Constants.LoaderConstants.loaderCutoffDistance,
              intake);
        }, loader);

    testAngleCommand = new RunCommand(
        () -> {
          launcherAngle.setAngle(30);
        }, launcherAngle);

    climberCommand = new RunCommand(
        () -> {
          double output = secondaryController.getLeftY();
          climber.run(output);
        }, climber);

    shooterFeed = new RunCommand(
        () -> {
          loader.run(0.2);
        }, loader);

    stopShooterComponents = new RunCommand(
        () -> {
          loader.run(0);
          launcher.setSpeed(0, 0);
        }, new Subsystem[] { loader, launcher });

    // Dumb launcher angle section
    AngleUpCommand = new RunCommand(
      () -> {
        dumbLauncherAngle.forceUp();
      }, dumbLauncherAngle
    );

    AngleDownCommand = new RunCommand(
      () -> {
        dumbLauncherAngle.forceDown();
      }, dumbLauncherAngle
    );

    AngleRelaxCommand = new RunCommand(
      () -> {
        dumbLauncherAngle.relax();
      }, dumbLauncherAngle
    );
    // ----------------------------

  }

  private void createSubsystems() {
    intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);

    launcher = new LauncherV2();
    launcherAngle = new LauncherAngle(Constants.LauncherAngleConstants.motorPort,
        Constants.LauncherAngleConstants.isReversed, Constants.LauncherAngleConstants.kP,
        Constants.LauncherAngleConstants.kI, Constants.LauncherAngleConstants.kD,
        Constants.LauncherAngleConstants.conversionFactor);

    distanceSensor = new DistanceSensor();
    loader = new LoaderV2(Constants.LoaderConstants.leftMotorPort,
        Constants.LoaderConstants.rightMotorPort, Constants.LoaderConstants.isReversed, distanceSensor);
    climber = new Climber();

    swerve = new Swerve();

    dumbLauncherAngle = new DumbLauncherAngle(
      Constants.DumbLauncherAngleConstants.motorPort,
      Constants.DumbLauncherAngleConstants.power
    );
  }

  private void configureBindings() {

    // reset gyro to 0 when y button is pressed, to be used when gyro drifts or need
    // different reference for field orientation

    // loads rings using distance sensor auto stop

    swerve.setDefaultCommand(swerveCmd);
    climber.setDefaultCommand(climberCommand);
    // driverController.y().whileTrue(shootCommand);
    driverController.y().onTrue(resetGyro);
    driverController.b().onTrue(new RepeatCommand(intakeCommand));
    driverController.x().onTrue(new LauncherIntake(distanceSensor, loader, launcher,
        Constants.LauncherIntakeConstants.theshold, Constants.LauncherIntakeConstants.speed));
    // driverController.rightBumper().whileTrue(loadCommand);

    secondaryController.a().onTrue(new VelocityLauncher(
        launcher,
        () -> {
          return Constants.LauncherConstants.LAUNCHER_LOW_REVS;
        },
        () -> {
          return Constants.LauncherConstants.LAUNCHER_LOW_REVS
              / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
        }));
    secondaryController.b().onTrue(new VelocityLauncher(
        launcher,
        () -> {
          return Constants.LauncherConstants.LAUNCHER_MED_REVS;
        },
        () -> {
          return Constants.LauncherConstants.LAUNCHER_MED_REVS
              / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
        }));
    secondaryController.y().onTrue(new VelocityLauncher(
        launcher,
        () -> {
          return Constants.LauncherConstants.LAUNCHER_HIGH_REVS;
        },
        () -> {
          return Constants.LauncherConstants.LAUNCHER_HIGH_REVS
              / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
        }));
    secondaryController.leftBumper().onTrue(shooterFeed);
    secondaryController.rightBumper().onTrue(stopShooterComponents);

    // Dumb launcher angle section
    secondaryController.b().whileTrue(AngleDownCommand);
    secondaryController.x().whileTrue(AngleUpCommand);
    dumbLauncherAngle.setDefaultCommand(AngleRelaxCommand);
    // ----------------------------
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
