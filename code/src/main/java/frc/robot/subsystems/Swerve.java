// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
  SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor, Constants.Mod0.canCoder);
  SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor, Constants.Mod1.canCoder);
  SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor, Constants.Mod2.canCoder);
  SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor, Constants.Mod3.canCoder);
  public Swerve() {
  
  }

  public void drive(double y, double x, double w, double gyro){ //TODO: switch for gyro
  Rotation2d gyroAngle = new Rotation2d(gyro);
  ChassisSpeeds speeds = new ChassisSpeeds(y, x, w);
  ChassisSpeeds frspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroAngle);
  SwerveModuleState[] states = kinematics.toSwerveModuleStates(frspeeds);
  
  Mod_0.setState(states[0]);
  Mod_1.setState(states[1]);
  Mod_2.setState(states[2]);
  Mod_3.setState(states[3]);
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
