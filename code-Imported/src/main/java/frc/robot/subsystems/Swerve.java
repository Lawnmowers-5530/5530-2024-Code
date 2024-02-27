// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Swerve extends SubsystemBase implements Loggable{

  PIDController rotationPID = new PIDController(Constants.RotationConstants.kP, Constants.RotationConstants.kI, Constants.RotationConstants.kD);

  SwerveDriveOdometry odometry;
  @Log
  boolean isUpdating;

  private static final SwerveModule Mod_0 = Constants.Modules.Mod_0;
  private static final SwerveModule Mod_1 = Constants.Modules.Mod_1;
  private static final SwerveModule Mod_2 = Constants.Modules.Mod_2;
  private static final SwerveModule Mod_3 = Constants.Modules.Mod_3;
  @Log
  String poseStr = "";
  @Log
  String output = "";

  @Log
  String robotRelativeSpeeds = "";

  SwerveModule[] modules = new SwerveModule[]{Mod_0, Mod_1, Mod_2, Mod_3};

  public Swerve() {
    SwerveModulePosition[] modPos = getModulePositions();
    odometry = new SwerveDriveOdometry(Constants.kinematics, Pgyro.getRot(), modPos);

    AutoBuilder.configureHolonomic(
    this::getPose,
    this::resetPose,
    this::getRobotRelativeSpeeds, //works
    this::autoDriveRobotRelative, //works
    new HolonomicPathFollowerConfig(
      Constants.translationConstants,
      Constants.rotationConstants,
      4.1, 
      Constants.driveBaseRadius,
      new ReplanningConfig()),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return false;
    },
      this
  );
  }

  public void drive(Vector2D vector, double omegaRadSec, boolean fieldRelative){

    Rotation2d gyroAngle = Pgyro.getRot();
    if(!fieldRelative){
      gyroAngle = Rotation2d.fromDegrees(0);
    }
    Vector2D rotated = VectorOperator.rotateVector2D(vector, gyroAngle);
    ChassisSpeeds speeds = new ChassisSpeeds(rotated.getvX(), rotated.getvY(), -omegaRadSec);

    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);
    Mod_0.setState(states[0]);
    Mod_1.setState(states[1]);
    Mod_2.setState(states[2]);
    Mod_3.setState(states[3]);

  }
  @Override
  public void periodic() {
    updateOdometry();
    poseStr = getPose().toString();
    robotRelativeSpeeds = this.getRobotRelativeSpeeds().toString();
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

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{Mod_0.getPos(), Mod_1.getPos(), Mod_2.getPos(), Mod_3.getPos()};
  }

  public void updateOdometry(){
    if(StaticLimeLight.getValidTarget()){
    odometry.resetPosition(Pgyro.getRot(), getModulePositions(), new Pose2d(StaticLimeLight.getPose2DBlue().getTranslation(), Pgyro.getRot()));
    isUpdating = true;
    }else{
      odometry.update(Pgyro.getRot(), getModulePositions());
      isUpdating = false;
    }
  }

  //chassis speeds consumer
  public void setChassisSpeeds(ChassisSpeeds speeds){
    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);
    Mod_0.setState(states[0]);
    Mod_1.setState(states[1]);
    Mod_2.setState(states[2]);
    Mod_3.setState(states[3]);
  }

  public void resetPose(Pose2d pose){
    odometry.resetPosition(Pgyro.getRot(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    ChassisSpeeds badSpeeds = Constants.kinematics.toChassisSpeeds(Mod_0.getState(), Mod_1.getState(), Mod_2.getState(), Mod_3.getState());
    return new ChassisSpeeds(-badSpeeds.vxMetersPerSecond, -badSpeeds.vyMetersPerSecond, badSpeeds.omegaRadiansPerSecond);
  }

  public void autoDriveRobotRelative(ChassisSpeeds speeds){
    Vector2D vector = new Vector2D(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, false);
    output = vector.toString();
    this.drive(vector, speeds.omegaRadiansPerSecond, false);
  }

  public void rotateToAngle(double angle){ //TODO
    double output = rotationPID.calculate(Pgyro.getRot().getDegrees(), angle);
    this.drive(new Vector2D(0, 0, false), output, false);
  }

}