// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.lib.Vector2D;
import frc.robot.module.limelight.Limelight;
import frc.robot.subsystems.Swerve;


public class RobotContainer implements Loggable{
  private final Swerve swerve = new Swerve();
  Limelight limelight = new Limelight();
  Trigger validTarget;
  PIDController angleController = new PIDController(0.25, 0, 0);

  @Log
  String blueLimePose = "a";

  @Log
  String x = "a";

  private final CommandXboxController driverController = new CommandXboxController(0);
  private Trigger Y;

  private final Command swerveCommand = new RunCommand(
    () -> {
      blueLimePose = limelight.testBotpose();

      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);
      Vector2D vector = new Vector2D(x, y, false);
      Y = driverController.y();
      swerve.drive(vector, w, Y);
}, swerve);

private final Command driveToVector = new RunCommand(
  () -> {
    double y=0.5;
    double x=0.5;
    double w=0;
    Vector2D vector = new Vector2D(x, -y, false);
    swerve.vectorDrive(vector, w);
}, swerve);

private final Command rotateToHdg = new RunCommand(
  () -> {
    double w=limelight.getHorizontalOffset();
    w = angleController.calculate(w);

    swerve.vectorDrive(new Vector2D(0, 0, false), w);
}, swerve);

  public RobotContainer() {
    configureBindings();
  }


  private void configureBindings() {
    swerve.setDefaultCommand(swerveCommand);
    driverController.x().whileTrue(driveToVector);

    
    BooleanSupplier isTargetValid = () -> limelight.hasValidTargets();
    validTarget = new Trigger(isTargetValid);

    driverController.b().and(validTarget).whileTrue(rotateToHdg);
    
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
