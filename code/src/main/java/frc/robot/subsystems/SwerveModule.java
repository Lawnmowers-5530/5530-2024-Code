// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenixpro.hardware.CANcoder;

public class SwerveModule extends SubsystemBase{
  private PIDController anglePID = new PIDController(0.0075, 0.0025, 0);
  @Log
  private double pidOut;
  private CANSparkMax drive;
  private CANSparkMax rotate;
  private RelativeEncoder encoder;
  private CANcoder canCoder;
  private double angleOffset;

  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double angleOffset) {
    drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    rotate = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    this.canCoder = new CANcoder(canCoderID);
    encoder = drive.getEncoder();
    encoder.setPosition(0);
    anglePID.enableContinuousInput(0, 360);
    this.angleOffset = angleOffset;
    
  }

  public void setState(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, new Rotation2d(getTurningPosition().getDegrees()*Math.PI/180));

        drive.set(state.speedMetersPerSecond/1.1);
        pidOut = anglePID.calculate(getTurningPosition().getDegrees(), state.angle.getDegrees());
        rotate.set(pidOut);
  }

  public Rotation2d getTurningPosition() {
    return new Rotation2d((this.canCoder.getAbsolutePosition().getValue()*360+this.angleOffset)*(Math.PI/180));
    }

  public double getVelocity() {
      return encoder.getVelocity();
    }
    
  public double getOffset(){
    return this.angleOffset;
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getDistance(){
    return encoder.getPosition()*(Math.PI*8);
  }

  public SwerveModulePosition getPos(){
    return new SwerveModulePosition(getDistance(), getTurningPosition());
  }
}