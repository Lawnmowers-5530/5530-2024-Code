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

  SwerveDriveOdometry odometry;

  SwerveModule Mod_0 = new SwerveModule(Constants.Mod0.driveMotor, Constants.Mod0.turnMotor, Constants.Mod0.canCoder, Constants.Mod0.angleOffset);
  SwerveModule Mod_1 = new SwerveModule(Constants.Mod1.driveMotor, Constants.Mod1.turnMotor, Constants.Mod1.canCoder, Constants.Mod1.angleOffset);
  SwerveModule Mod_2 = new SwerveModule(Constants.Mod2.driveMotor, Constants.Mod2.turnMotor, Constants.Mod2.canCoder, Constants.Mod2.angleOffset);
  SwerveModule Mod_3 = new SwerveModule(Constants.Mod3.driveMotor, Constants.Mod3.turnMotor, Constants.Mod3.canCoder, Constants.Mod3.angleOffset);

  SwerveModule[] modules = new SwerveModule[]{Mod_0, Mod_1, Mod_2, Mod_3};

  public Swerve() {
    SwerveModulePosition[] modPos = getModulePositions();
    odometry = new SwerveDriveOdometry(Constants.kinematics, new Rotation2d(Pgyro.getGyro().getYaw().getValue()*(Math.PI/180)), modPos);
  }

  public void drive(double y, double x, double w, Trigger Y){
    if(Y.getAsBoolean()){
      Pgyro.zeroGyro();
    }

    Rotation2d gyroAngle = Pgyro.getRot();

    double[] rotated = rotateVector(x, y, gyroAngle);
    
    double xn = rotated[0];
    double yn = rotated[1];

    ChassisSpeeds speeds = new ChassisSpeeds(yn, xn, w);

    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);
    Mod_0.setState(states[0]);
    Mod_1.setState(states[1]);
    Mod_2.setState(states[2]);
    Mod_3.setState(states[3]);

  }
  @Override
  public void periodic() {
    odometry.update(Pgyro.getRot(), getModulePositions());
  }

  public double[] rotateVector(double x, double y, Rotation2d angle){
  return new double[]{(x*angle.getCos())-(y*angle.getSin()), (x*angle.getSin())+(y*angle.getCos())};
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void autonDrive(double xSpeed, double ySpeed, double rot) {
    SwerveModuleState[] swerveModuleStates =
        Constants.kinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot));
    setModuleStates(swerveModuleStates);
  }


  public void setModuleStates(SwerveModuleState[] desiredStates) {
    Mod_0.setState(desiredStates[0]);
    Mod_1.setState(desiredStates[1]);
    Mod_2.setState(desiredStates[2]);
    Mod_3.setState(desiredStates[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{Mod_0.getPos(), Mod_1.getPos(), Mod_2.getPos(), Mod_3.getPos()};
  }
}