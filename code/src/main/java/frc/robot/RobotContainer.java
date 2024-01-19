// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;

import frc.robot.subsystems.StaticLimeLight;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.shooterTest;


public class RobotContainer implements Loggable{
  private final Swerve swerve = new Swerve();
  private final shooterTest shooter = new shooterTest();
  Trigger validTarget;
  PIDController angleController = new PIDController(0.15, 0.05, 0.0);
  PIDController limelightAngleController = new PIDController(0.025, 0.01, 0.0);

  @Log
  double limeOff;
  double rads;
  double w;
  double limeAngle;
  String x = "a";

  Pose2d currentPose;
  Pose2d goalPose;

  @Log
  String currentPoseStr = "a";
  @Log
  String goalPoseStr = "a";

  @Config
  double poseX = 14;
  @Config
  double poseY = 3.5;
  @Config
  double poseRotGoal = 0;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private Trigger Y;

  private final Command swerveCommand = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);

      if(driverController.a().getAsBoolean()==true){
        x/=2;
        y/=2;
        w/=2;
      }
      rads = Pgyro.getHdgRad();

      Vector2D vector = new Vector2D(y, x, false);
      Y = driverController.y();
      swerve.drive(vector, w, Y);
      currentPoseStr = swerve.getPoseOdometry().toString();

}, swerve);

private final Command driveToPose = new RunCommand(
  () -> {
    goalPose = new Pose2d(new Translation2d(poseX, poseY), new Rotation2d(poseRotGoal));
    currentPose = swerve.getPoseOdometry();
    currentPoseStr = currentPose.toString();
    goalPoseStr = goalPose.toString();
    Vector2D vector = VectorOperator.fromPose(currentPose, goalPose);
    w = -angleController.calculate(currentPose.getRotation().getRadians(), poseRotGoal);
    swerve.vectorDrive(vector, w);
}, swerve);

private final Command rotateToHdg = new RunCommand(
  () -> {
    limeOff=StaticLimeLight.getHorizontalOffset();
    limeAngle = -limelightAngleController.calculate(limeOff);
    //limeAngle = limelight.getHorizontalOffset()/10;

    swerve.vectorDrive(new Vector2D(0, 0, false), limeAngle);
}, swerve);

private final Command shooterCommand = new RunCommand(
  () -> {
    double speed = driverController.getRightY();
    shooter.runShooter(speed);
  }, shooter);

  public RobotContainer() {
    angleController.enableContinuousInput(0, Math.PI*2);
    configureBindings();
  }


  private void configureBindings() {
    //swerve.setDefaultCommand(swerveCommand);
    //validTarget = new Trigger(StaticLimeLight.validTargetSupp());
    ////limeTest.setDefaultCommand(limeCommand);
    //driverController.x().whileTrue(driveToPose);
    //driverController.b().and(validTarget).whileTrue(rotateToHdg);
    shooter.setDefaultCommand(shooterCommand);

    
  }
  
  public Command getAutonomousCommand() {
    var thetaController =
    new ProfiledPIDController(
        0.1, 0, 0, Constants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
  SwerveControllerCommand swerveControllerCommand =
  new SwerveControllerCommand(
      Trajectories.trajectory,
      swerve::getPose,
      Constants.kinematics,

      // Position controllers
      new PIDController(0.1, 0, 0),
      new PIDController(0.1, 0, 0),
      thetaController,
      swerve::setModuleStates,
      swerve);
    return swerveControllerCommand.andThen(() -> swerve.autonDrive(0, 0, 0));
}

}
