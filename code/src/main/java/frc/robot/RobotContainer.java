// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.management.InstanceNotFoundException;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Pigeon2 pigeon = new Pigeon2(17);
  private Trigger Y;
  private final Runnable odoInit = new Runnable(){
    @Override
    public void run(){
      swerve.createOdometry();
    }
  };

  private final Command odometryInitializer = new InstantCommand(odoInit, swerve);
  private final Command swerveCommand = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.15);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.15);
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.15);
      Y = driverController.y();
      swerve.drive(y, x, w, pigeon, Y);
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
    return swerveControllerCommand.andThen(() -> swerve.autonDrive(0, 0, 0, pigeon));
}
}
