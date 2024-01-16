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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.module.limelight.Limelight;
import frc.robot.subsystems.LimeTest;
import frc.robot.subsystems.Pgyro;
import frc.robot.subsystems.Swerve;


public class RobotContainer implements Loggable{
  private final Swerve swerve = new Swerve();
  private final LimeTest limeTest = new LimeTest();

  Limelight limelight = new Limelight();
  Trigger validTarget;
  PIDController angleController = new PIDController(0.15, 0.05, 0.175);
  PIDController limelightAngleController = new PIDController(0.025, 0.01, 0.0);

  @Log
  double limeOff;

  @Log
  double goalAngle;

  double rads;

  double w;
  
  @Log
  double limeAngle;


  @Log
  String blueLimePose = "a";

  @Log
  String x = "a";

  private final CommandXboxController driverController = new CommandXboxController(0);
  private Trigger Y;

  private final Command swerveCommand = new RunCommand(
    () -> {
      blueLimePose = limelight.testBotpose();
      boolean slow;

      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);

      if(driverController.a().getAsBoolean()==true){
        x/=2;
        y/=2;
        w/=2;
      }
      goalAngle+=w*0.1;
      rads = Pgyro.getHdgRad();
      //w = -angleController.calculate(rads, goalAngle);

      Vector2D vector = new Vector2D(y, x, false);
      Y = driverController.y();
      swerve.drive(vector, w, Y);
}, swerve);

private final Command driveToPose = new RunCommand(
  () -> {
    w = 0;
    Pose2d goalPose = new Pose2d(new Translation2d(1, 1), new Rotation2d(w));
    Vector2D vector = VectorOperator.fromPose(swerve.getPoseOdometry(), goalPose);
    w = -angleController.calculate(rads, w);
    swerve.vectorDrive(vector, w);
}, swerve);

private final Command rotateToHdg = new RunCommand(
  () -> {
    if(limelight.hasValidTargets()){
    limeOff=limelight.getHorizontalOffset();
    limeAngle = -limelightAngleController.calculate(limeOff);
    //limeAngle = limelight.getHorizontalOffset()/10;

    swerve.vectorDrive(new Vector2D(0, 0, false), limeAngle);
    }
}, swerve);

private final Command limeOdometry = new RunCommand(
  () -> {
    swerve.updateOdometry(limelight);
  }, swerve
);



  public RobotContainer() {
    goalAngle = 0;
    angleController.enableContinuousInput(0, Math.PI);
    configureBindings();
  }


  private void configureBindings() {
    swerve.setDefaultCommand(swerveCommand);
    //limeTest.setDefaultCommand(limeCommand);
    driverController.x().whileTrue(driveToPose);

    
    BooleanSupplier isTargetValid = () -> limelight.hasValidTargets();
    validTarget = new Trigger(isTargetValid);
    //validTarget.whileTrue(limeOdometry);
    driverController.a().whileTrue(rotateToHdg);
    
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
