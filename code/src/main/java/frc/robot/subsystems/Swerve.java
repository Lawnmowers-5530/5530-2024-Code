// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.Constants;
import frc.robot.data.GlobalState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Swerve extends SubsystemBase implements Loggable {

  PIDController rotationPID;

  SwerveDriveOdometry odometry;
  @Log
  boolean isUpdating;
  private boolean isCoasting;
  private static final SwerveModule Mod_0 = Constants.Modules.Mod_0;
  private static final SwerveModule Mod_1 = Constants.Modules.Mod_1;
  private static final SwerveModule Mod_2 = Constants.Modules.Mod_2;
  private static final SwerveModule Mod_3 = Constants.Modules.Mod_3;
  @Log
  String poseStr = "";

  double rotationOutput;

  @Log
  String robotRelativeSpeeds = "";

  private SwerveModuleState[] states;

  public Swerve() {
    rotationPID = new PIDController(Constants.RotationConstants.kP, Constants.RotationConstants.kI,
        Constants.RotationConstants.kD);
    rotationPID.setTolerance(2);
    SwerveModulePosition[] modPos = getModulePositions();
    odometry = new SwerveDriveOdometry(Constants.kinematics, Pgyro.getRot(), modPos);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds, // works
        this::autoDriveRobotRelative, // works
        new HolonomicPathFollowerConfig(
            Constants.PathPlannerConstants.translationConstants,
            Constants.PathPlannerConstants.rotationConstants,
            3.8,
            Constants.driveBaseRadius,
            new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public void drive(Vector2D vector, double omegaRadSec, boolean fieldRelative) {

    Rotation2d gyroAngle = Pgyro.getRot();
    Vector2D rotated;
    if (fieldRelative) {
      rotated = VectorOperator.rotateVector2D(vector, gyroAngle);
    } else {
      rotated = vector;
    }
    ChassisSpeeds speeds = new ChassisSpeeds(rotated.getvX(), rotated.getvY(), -omegaRadSec);

    states = Constants.kinematics.toSwerveModuleStates(speeds);
    Mod_0.setState(states[0]);
    Mod_1.setState(states[1]);
    Mod_2.setState(states[2]);
    Mod_3.setState(states[3]);

  }

  @Override
  public void periodic() {
    if (isCoasting && GlobalState.isEnabled) {
      Mod_0.setIdleMode(IdleMode.kBrake);
      Mod_1.setIdleMode(IdleMode.kBrake);
      Mod_2.setIdleMode(IdleMode.kBrake);
      Mod_3.setIdleMode(IdleMode.kBrake);
      isCoasting = false;
    }
    updateOdometry();
    poseStr = getPose().toString();
    robotRelativeSpeeds = this.getRobotRelativeSpeeds().toString();

    SmartDashboard.putString("robot rel speeds", getRobotRelativeSpeeds().toString());
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

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { Mod_0.getPos(), Mod_1.getPos(), Mod_2.getPos(), Mod_3.getPos() };
  }

  public void updateOdometry() {
    if (StaticLimeLight.getValidTarget()) {
      odometry.resetPosition(Pgyro.getRot(), getModulePositions(),
          new Pose2d(StaticLimeLight.getPose2DBlue().getTranslation(), Pgyro.getRot()));
      isUpdating = true;
    } else {
      odometry.update(Pgyro.getRot(), getModulePositions());
      isUpdating = false;
    }
  }

  // chassis speeds consumer
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    states = Constants.kinematics.toSwerveModuleStates(speeds);
    Mod_0.setState(states[0]);
    Mod_1.setState(states[1]);
    Mod_2.setState(states[2]);
    Mod_3.setState(states[3]);
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(Pgyro.getRot(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds badSpeeds = Constants.kinematics.toChassisSpeeds(Mod_0.getState(), Mod_1.getState(), Mod_2.getState(),
        Mod_3.getState());
    return new ChassisSpeeds(-badSpeeds.vxMetersPerSecond, -badSpeeds.vyMetersPerSecond,
        badSpeeds.omegaRadiansPerSecond);
  }

  public void autoDriveRobotRelative(ChassisSpeeds speeds) {
    SmartDashboard.putString("drive goal", speeds.toString());
    Vector2D vector = new Vector2D(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, false);
    this.drive(vector, speeds.omegaRadiansPerSecond, false);
  }

  public void rotateToAngle(double angle) {
    rotationOutput = rotationPID.calculate(Pgyro.getRot().getDegrees(), angle);
    this.drive(new Vector2D(0, 0, false), rotationOutput, false);
  }

  public Command pathFind(Pose2d endPose) {
    return AutoBuilder.pathfindToPose(endPose, Constants.PathPlannerConstants.constraints);
  }

  public boolean atTargetAngle() {
    return rotationPID.atSetpoint();
  }

  public void disabledPeriodic() {
    if (!isCoasting) {
      isCoasting = true;
      Mod_0.setIdleMode(IdleMode.kCoast);
      Mod_1.setIdleMode(IdleMode.kCoast);
      Mod_2.setIdleMode(IdleMode.kCoast);
      Mod_3.setIdleMode(IdleMode.kCoast);
    }
  }

}