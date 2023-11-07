// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule extends SubsystemBase {

  PIDController anglePID = new PIDController(.5, .1, 0);

  CANSparkMax drive;
  CANSparkMax rotate;
  RelativeEncoder encoder;

  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID) {
    drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    rotate = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    CANCoder canCoder = new CANCoder(canCoderID);
    encoder = rotate.getEncoder();
    encoder.setPosition(canCoder.getAbsolutePosition());
  }
  public void setState(SwerveModuleState state){
      state = SwerveModuleState.optimize(state, new Rotation2d(encoder.getPosition()));
      
        drive.set(state.speedMetersPerSecond);
        rotate.set(anglePID.calculate(encoder.getPosition(), state.angle.getDegrees()));
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
