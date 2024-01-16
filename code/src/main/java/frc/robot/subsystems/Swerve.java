// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.Constants;
import frc.robot.module.limelight.Limelight;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class Swerve extends SubsystemBase implements Loggable{

  SwerveDriveOdometry odometry;
  
  @Log
  String pose = "a";

  private static final SwerveModule Mod_0 = Constants.Modules.Mod_0;
  private static final SwerveModule Mod_1 = Constants.Modules.Mod_1;
  private static final SwerveModule Mod_2 = Constants.Modules.Mod_2;
  private static final SwerveModule Mod_3 = Constants.Modules.Mod_3;

  SwerveModule[] modules = new SwerveModule[]{Mod_0, Mod_1, Mod_2, Mod_3};

  public Swerve() {
    SwerveModulePosition[] modPos = getModulePositions();
    odometry = new SwerveDriveOdometry(Constants.kinematics, Pgyro.getRot(), modPos);
  }

  public void drive(Vector2D vector, double omegaRadSec, Trigger Y){
    if(Y!=null){
    if(Y.getAsBoolean()){
      Pgyro.zeroGyro();
    }
  }

    Rotation2d gyroAngle = Pgyro.getRot();

    Vector2D rotated = VectorOperator.rotateVector2D(vector, gyroAngle);

    ChassisSpeeds speeds = new ChassisSpeeds(rotated.getvX(), rotated.getvY(), omegaRadSec);

    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);
    Mod_0.setState(states[0]);
    Mod_1.setState(states[1]);
    Mod_2.setState(states[2]);
    Mod_3.setState(states[3]);

  }
  @Override
  public void periodic() {
    odometry.update(Pgyro.getRot(), getModulePositions());
    pose = odometry.getPoseMeters().toString();
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

  public void vectorDrive(Vector2D target, double thetaRadSec){
    this.drive(target, thetaRadSec, null);
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

  public void updateOdometry(Limelight limelight){
    odometry.resetPosition(Pgyro.getRot(), getModulePositions(), limelight.getPose2DBlue());
  }

  public Pose2d getPoseOdometry(){
    return odometry.getPoseMeters();
  }
}