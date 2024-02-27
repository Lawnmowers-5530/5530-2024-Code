// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import frc.lib.Shot;
import frc.lib.ShotCalculator;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.subsystems.StaticLimeLight;
import frc.robot.commands.LauncherIntake;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherAngle;
import frc.robot.subsystems.LauncherV2;
import frc.robot.subsystems.LoaderV2;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

public class RobotContainer implements Loggable{
  private Field2d field;
  private SendableChooser<Command> autoChooser;
  private Swerve swerve = new Swerve();
  private DistanceSensor distanceSensor;
  private LoaderV2 loader;
  private LauncherAngle launcherAngle;

  private LauncherV2 launcher;

  private Trigger validTarget;
  private DoubleSupplier shotAngleSupplier;

  private CommandXboxController driverController;

  private Intake intake;

  public RobotContainer() {
    distanceSensor = new DistanceSensor();
    loader = new LoaderV2(Constants.LoaderConstants.leftMotorPort, Constants.LoaderConstants.rightMotorPort, Constants.LoaderConstants.isReversed, distanceSensor);
    launcherAngle = new LauncherAngle(Constants.LauncherAngleConstants.motorPort, Constants.LauncherAngleConstants.isReversed, Constants.LauncherAngleConstants.kP, Constants.LauncherAngleConstants.kI, Constants.LauncherAngleConstants.kD, Constants.LauncherAngleConstants.conversionFactor);
    launcher = new LauncherV2();
    intake = new Intake(Constants.IntakeConstants.motorPort, Constants.IntakeConstants.isReversed);
    validTarget = new Trigger(StaticLimeLight.validTargetSupp());

    NamedCommands.registerCommand("shoot", shootCommand);

    shotAngleSupplier = new DoubleSupplier() {
    @Override
    public double getAsDouble() {
      return ShotCalculator.angleToTarget(swerve.getPose());
    }
  };

    driverController = new CommandXboxController(0);
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
  
    SmartDashboard.putData("Auton chooser", autoChooser);

    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

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




  //normal speed swerve drive command
  private final Command swerveCmd = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);

      Vector2D vector = new Vector2D(y, x, false);
      swerve.drive(vector, -w, true);

}, swerve
);

  //slow speed swerve drive command
  private final Command swerveSlowCmd = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15)/2;
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15)/2;
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15)/2;

      Vector2D vector = new Vector2D(y, x, false);
      swerve.drive(vector, -w, true);

}, swerve
);

  //swerve drive command that switches between normal and slow command based on the a button
  private final Command swerveCommand = new ConditionalCommand(
    swerveCmd,
    swerveSlowCmd,
    driverController.a()
);

  //reset gyro to 0 when y button is pressed, to be used when gyro drifts or need different reference for field orientation
  private final Command resetGyro = new RunCommand(
    () -> {
      Pgyro.zeroGyro();
}, new Subsystem[]{}
);

  //loads rings using distance sensor auto stop
  private final Command loadCommand = new RunCommand(
  () -> {
    loader.runUntilBeamBreak(Constants.LoaderConstants.loaderSpeed, Constants.LoaderConstants.loaderCutoffDistance);
}, loader
);

  private final Command shootCommand = new RunCommand(
    () -> {
      Pose2d currentPose = swerve.getPose();

      //find field oriented vector of robot
      Vector2D robotRelativeVector = new Vector2D(swerve.getRobotRelativeSpeeds().vxMetersPerSecond, swerve.getRobotRelativeSpeeds().vyMetersPerSecond, false);
      Vector2D robotVector = VectorOperator.rotateVector2D(robotRelativeVector, Pgyro.getRot());

      //find distance and angle to target
      double distToTarget = Constants.targetTranslation.getDistance(currentPose.getTranslation());
      double angleToTarget = shotAngleSupplier.getAsDouble();

      //use shooter library to calculate final shot vector
      Shot shot = ShotCalculator.vecFinal(robotVector, distToTarget, angleToTarget);
      
      //shoot the calculated shot
      double leftSpeed = shot.getSpeed();
      double rightSpeed = shot.getSpeed(); //TODO

      launcher.setVelocity(leftSpeed, rightSpeed);
      launcherAngle.setAngle(shot.getThetaDeg());

      loader.run(Constants.LauncherConstants.loaderShotSpeed);
      
      swerve.rotateToAngle(Math.toDegrees(shot.getPhiDeg()));

      //temp logging
      SmartDashboard.putNumber("distToTarget", distToTarget);
      SmartDashboard.putNumber("angleToTarget", angleToTarget);
      SmartDashboard.putString("currentPose", currentPose.toString());
      SmartDashboard.putString("robotVector", robotVector.toString());
      SmartDashboard.putString("shot", shot.toString());
}, new Subsystem[]{launcher, launcherAngle}
);

  private Command intakeCommand = new RunCommand(
    () -> {
      intake.run(Constants.IntakeConstants.intakeSpeed);
      loader.runUntilBeamBreak(Constants.LoaderConstants.loaderSpeed, Constants.LoaderConstants.loaderCutoffDistance);
}, new Subsystem[]{intake, loader}
);
  private Command testAngleCommand = new RunCommand(
    () -> {
      launcherAngle.setAngle(0.1);
}, launcherAngle
);

  private Command ampScore = AutoBuilder.pathfindToPose(
    new Pose2d(14.5, 7.5, new Rotation2d(Math.PI/2)),
    new PathConstraints(4.1, 1, 2, 1)
);

  private void configureBindings() {
    swerve.setDefaultCommand(swerveCommand);
    //driverController.x().whileTrue(shootCommand);
    driverController.y().whileTrue(resetGyro);
    driverController.x().whileTrue(testAngleCommand);
    driverController.b().whileTrue(intakeCommand);
    driverController.a().onTrue(new LauncherIntake(distanceSensor, loader, launcher, Constants.LauncherIntakeConstants.theshold, Constants.LauncherIntakeConstants.speed));
 
    //driverController.rightBumper().whileTrue(loadCommand);
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
