// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import frc.robot.subsystems.Swerve;


public class RobotContainer implements Loggable{
  private final Swerve swerve = new Swerve();

  private final CommandXboxController driverController = new CommandXboxController(0);
  private Trigger Y;
  private final Runnable odoInit = new Runnable(){
    @Override
    public void run(){
      //method call here;
    }
  };

  private final Command odometryInitializer = new InstantCommand(odoInit, swerve);
  private final Command swerveCommand = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);
      Y = driverController.y();
      swerve.drive(y, x, w, Y);
}, swerve);

private final Command driveToVector = new RunCommand(
  () -> {
    double y=1;
    double x=1;
    double w=1;
    swerve.drive(y, x, w, false);
}, swerve);

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    configureBindings();
  }
  public void updateLogger() {
    Logger.updateEntries();
  }
  private void configureBindings() {
    swerve.setDefaultCommand(swerveCommand);
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
