// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  Translation2d m0 = new Translation2d(Constants.trackWidth/2, Constants.wheelBase/2);
  Translation2d m1 = new Translation2d(Constants.trackWidth/2, -Constants.wheelBase/2);
  Translation2d m2 = new Translation2d(-Constants.trackWidth/2, Constants.wheelBase/2);
  Translation2d m3 = new Translation2d(-Constants.trackWidth/2, -Constants.wheelBase/2);


  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m0, m1, m2, m3);

  SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor, Constants.Mod0.canCoder, Constants.Mod0.angleOffset);
  SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor, Constants.Mod1.canCoder, Constants.Mod1.angleOffset);
  SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor, Constants.Mod2.canCoder, Constants.Mod2.angleOffset);
  SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor, Constants.Mod3.canCoder, Constants.Mod3.angleOffset);
  public Swerve() {
  }

  public void drive(double y, double x, double w, Pigeon2 gyro, Trigger Y){
  if(Y.getAsBoolean()){
    zeroGyro(gyro);
  }
  double yaw = gyro.getYaw().getValue();
  Rotation2d gyroAngle = new Rotation2d(-(Math.abs(yaw)%360)/57.2958); //gyro abs angle in rads
  double[] rotated = rotateVector(x, y, gyroAngle);
  
  double xn = rotated[0];
  double yn = rotated[1];

  ChassisSpeeds speeds = new ChassisSpeeds(yn, xn, w);
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

  public double[] rotateVector(double x, double y, Rotation2d gyroAngle){
  return new double[]{(x*gyroAngle.getCos())-(y*gyroAngle.getSin()), (x*gyroAngle.getSin())+(y*gyroAngle.getCos())};
  }

  public void zeroGyro(Pigeon2 pig){
    pig.setYaw(0);
  }
}