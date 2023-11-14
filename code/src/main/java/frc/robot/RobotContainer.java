// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMUConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;

import frc.robot.subsystems.Swerve;


public class RobotContainer implements Loggable{
  private final Swerve swerve = new Swerve();
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final PigeonIMU pigeon = new PigeonIMU(17);
  private final Command swerveCommand = new RunCommand(
    () -> {
      double y = MathUtil.applyDeadband(driverController.getLeftY(), 0.09);
      double x = MathUtil.applyDeadband(driverController.getLeftX(), 0.09);
      double w = MathUtil.applyDeadband(driverController.getRightX(), 0.09);
      swerve.drive(y, x, w, pigeon);
}, swerve);
  public RobotContainer() {
    pigeon.configFactoryDefault();
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
    return Commands.print("No autonomous command configured");
  }
}
