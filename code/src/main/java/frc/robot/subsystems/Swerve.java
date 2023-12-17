// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenixpro.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  SwerveDriveOdometry odometry;
  public void createOdometry(Pigeon2 pigeon){
    SwerveModulePosition[] modPos = new SwerveModulePosition[]{Mod_0.getPos(), Mod_1.getPos(), Mod_2.getPos(), Mod_3.getPos()};
    odometry = new SwerveDriveOdometry(Constants.kinematics, new Rotation2d(pigeon.getYaw().getValue()*(Math.PI/180)), modPos);
  }

  SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor, Constants.Mod0.canCoder, Constants.Mod0.angleOffset);
  SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor, Constants.Mod1.canCoder, Constants.Mod1.angleOffset);
  SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor, Constants.Mod2.canCoder, Constants.Mod2.angleOffset);
  SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor, Constants.Mod3.canCoder, Constants.Mod3.angleOffset);
  SwerveModule[] modules = new SwerveModule[]{Mod_0, Mod_1, Mod_2, Mod_3};
  public Swerve() {
  }
  int x = 0;
  public void drive(double y, double x, double w, Pigeon2 gyro, Trigger Y){
  if(Y.getAsBoolean()){
    zeroGyro(gyro);
  }
  double yaw = gyro.getYaw().getValue();
  System.out.println("yaw: "+ yaw);
  //if(yaw<0){
  //  yaw = Math.abs((360+yaw))%360;
  //}

  Rotation2d gyroAngle = new Rotation2d(-(Math.abs(yaw)%360)/57.2958); //gyro abs angle in rads
  double[] rotated = rotateVector(x, y, gyroAngle);
  //if(x==0){
  //  createOdometry(new Rotation2d(yaw*(Math.PI/180)));
  //  x+=1;
  //}
  //odometry.update(gyroAngle, new SwerveModulePosition[]{Mod_0.getPos(), Mod_1.getPos(), Mod_2.getPos(), Mod_3.getPos()});
  double xn = rotated[0];
  double yn = rotated[1];

  ChassisSpeeds speeds = new ChassisSpeeds(yn, xn, w);
  //ChassisSpeeds frspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyroAngle);
  SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);
  Mod_0.setState(states[0]);
  Mod_1.setState(states[1]);
  Mod_2.setState(states[2]);
  Mod_3.setState(states[3]);
  
  }

  public void autonDrive(double xSpeed, double ySpeed, double rot, Pigeon2 pigeon) {
    SwerveModuleState[] swerveModuleStates =
        Constants.kinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }

  public void autonDrive2() {
    SwerveModuleState state = new SwerveModuleState(-0.5, new Rotation2d(0));
    SwerveModuleState stopState = new SwerveModuleState(0.0, new Rotation2d(0));
    for(SwerveModule mod: modules){
      if(Math.abs(mod.getDistance())<115){
        mod.setState(state);
      }else{
        mod.setState(stopState);
      }

    } 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] rotateVector(double x, double y, Rotation2d gyroAngle){
  return new double[]{(x*gyroAngle.getCos())-(y*gyroAngle.getSin()), (x*gyroAngle.getSin())+(y*gyroAngle.getCos())};
  }

  public void zeroGyro(Pigeon2 pig){
    pig.setYaw(-36000);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    Mod_0.setState(desiredStates[0]);
    Mod_1.setState(desiredStates[1]);
    Mod_2.setState(desiredStates[2]);
    Mod_3.setState(desiredStates[3]);
  }
}