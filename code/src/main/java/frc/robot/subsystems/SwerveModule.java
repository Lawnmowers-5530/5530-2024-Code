// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule extends SubsystemBase{
  PIDController anglePID = new PIDController(0.5, 0, 0);
  @Log
  private double pidOut;
  CANSparkMax drive;
  CANSparkMax rotate;
  RelativeEncoder encoder;
  CANCoder canCoder;

  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID) {
    drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    rotate = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    CANCoder canCoder = new CANCoder(canCoderID);
    encoder = rotate.getEncoder();
    encoder.setPosition(canCoder.getAbsolutePosition());
    anglePID.enableContinuousInput(-Math.PI, Math.PI); //its a wheel
    encoder.setPositionConversionFactor(360); //convert rotations to degrees
    
  }
  public void setState(SwerveModuleState state){
      state = SwerveModuleState.optimize(state, new Rotation2d(canCoder.getAbsolutePosition()));

        drive.set(state.speedMetersPerSecond/2.5);
        pidOut = anglePID.calculate(encoder.getPosition(), state.angle.getDegrees());
        rotate.set(pidOut);
  }

  public double getTurningPosition() {
      return encoder.getPosition();
      //return getAbsoluteEncoderRad();
    }
  public double getTurningVelocity() {
      return encoder.getVelocity();
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
