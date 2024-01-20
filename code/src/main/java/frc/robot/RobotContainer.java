// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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


public class RobotContainer implements Loggable{
  private final Swerve swerve = new Swerve();
  Trigger validTarget;
  PIDController angleController = new PIDController(0.65, 0.05, 0.0);
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
  double poseX = 13;
  @Config
  double poseY = 4;
  @Config
  double poseRotGoal = 0;

  private final CommandXboxController driverController = new CommandXboxController(0);

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
      swerve.drive(vector, w);
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
    swerve.vectorDrive(new Vector2D(vector.getvX(), vector.getvY(), false), w);
}, swerve);

private final Command rotateToHdg = new RunCommand(
  () -> {
    limeOff=StaticLimeLight.getHorizontalOffset();
    limeAngle = -limelightAngleController.calculate(limeOff);
    //limeAngle = limelight.getHorizontalOffset()/10;

    swerve.vectorDrive(new Vector2D(0, 0, false), limeAngle);
}, swerve);

private final Command resetGyro = new RunCommand(
  () -> {
    Pgyro.zeroGyro();
  }, new Subsystem[]{} );

  public RobotContainer() {
    angleController.enableContinuousInput(0, Math.PI*2);
    configureBindings();
  }


  private void configureBindings() {
    swerve.setDefaultCommand(swerveCommand);
    validTarget = new Trigger(StaticLimeLight.validTargetSupp());
    //limeTest.setDefaultCommand(limeCommand);
    driverController.x().whileTrue(driveToPose);
    driverController.b().and(validTarget).whileTrue(rotateToHdg);

    driverController.y().whileTrue(resetGyro);

    
  }
  
  public Command getAutonomousCommand() {
    return null;
}

}
