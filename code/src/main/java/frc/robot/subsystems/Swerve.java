// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import frc.lib.Vector2D;
import frc.lib.VectorOperator;
import frc.robot.Constants;
import frc.robot.data.GlobalState;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Swerve extends SubsystemBase implements Loggable {

  private LimelightHelpers.PoseEstimate limelightMeasurement;

  private SwerveDrivePoseEstimator poseEstimator;

  private PIDController rotationPID;

  @Log
  boolean isUpdating;

  private boolean isCoasting;
  private static final SwerveModule Mod_0 = Constants.Modules.Mod_0;
  private static final SwerveModule Mod_1 = Constants.Modules.Mod_1;
  private static final SwerveModule Mod_2 = Constants.Modules.Mod_2;
  private static final SwerveModule Mod_3 = Constants.Modules.Mod_3;
  
  @Log
  String poseStr = "";

  private double rotationOutput;

  private SwerveModuleState[] states;

  private BooleanSupplier sideSupplier = () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

  public Swerve() {
    poseEstimator = new SwerveDrivePoseEstimator(Constants.kinematics, Pgyro.getRot(), getModulePositions(), new Pose2d());
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, Rotation2d.fromDegrees(45).getRadians())); //set vision measurement trust tolerance to 1 meter and 45 degrees

    rotationPID = new PIDController(Constants.RotationConstants.kP, Constants.RotationConstants.kI,
        Constants.RotationConstants.kD);
    rotationPID.setTolerance(2);

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
        sideSupplier,
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
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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
    if(sideSupplier.getAsBoolean()){
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");
    }else{
      limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    }
    if(limelightMeasurement.tagCount >= 2){
    poseEstimator.addVisionMeasurement(limelightMeasurement.pose, limelightMeasurement.timestampSeconds);
    }
    
    poseEstimator.update(Pgyro.getRot(), getModulePositions());
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
    poseEstimator.resetPosition(Pgyro.getRot(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds badSpeeds = Constants.kinematics.toChassisSpeeds(Mod_0.getState(), Mod_1.getState(), Mod_2.getState(),
        Mod_3.getState());
    return new ChassisSpeeds(-badSpeeds.vxMetersPerSecond, -badSpeeds.vyMetersPerSecond,
        badSpeeds.omegaRadiansPerSecond);
  }

  public Vector2D getFieldRelativeSpeeds() {
    return VectorOperator.rotateVector2D(new Vector2D(this.getRobotRelativeSpeeds().vxMetersPerSecond, this.getRobotRelativeSpeeds().vyMetersPerSecond, false), Pgyro.getRot().times(-1));
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
    return false;//rotationPID.atSetpoint();
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