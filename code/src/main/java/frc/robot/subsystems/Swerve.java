// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  Translation2d m0 = new Translation2d(Constants.trackWidth/2, Constants.wheelBase/2);
  Translation2d m1 = new Translation2d(Constants.trackWidth/2, -Constants.wheelBase/2);
  Translation2d m2 = new Translation2d(-Constants.trackWidth/2, Constants.wheelBase/2);
  Translation2d m3 = new Translation2d(-Constants.trackWidth/2, -Constants.wheelBase/2);
  //test!

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m0, m1, m2, m3);
  //im literally making changes right now hello?
  SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor, Constants.Mod0.canCoder, Constants.Mod0.angleOffset);
  SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor, Constants.Mod1.canCoder, Constants.Mod1.angleOffset);
  SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor, Constants.Mod2.canCoder, Constants.Mod2.angleOffset);
  SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor, Constants.Mod3.canCoder, Constants.Mod3.angleOffset);
  public Swerve() {
  }

  public void drive(double y, double x, double w, Pigeon2 gyro){
  //Rotation2d gyroAngle = new Rotation2d((Math.abs(gyro.getYaw().getValue())%360)/57.2958); //gyro abs angle in rads
  //System.out.println(gyroAngle);
  ChassisSpeeds speeds = new ChassisSpeeds(y, x, w);
  //ChassisSpeeds frspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroAngle);
  SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
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