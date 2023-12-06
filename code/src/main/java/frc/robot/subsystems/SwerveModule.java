// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule extends SubsystemBase{
  private PIDController anglePID = new PIDController(0.005, 0, 0);
  @Log
  private double pidOut;
  private CANSparkMax drive;
  private CANSparkMax rotate;
  private RelativeEncoder encoder;
  private CANCoder canCoder;
  private double angleOffset;

  public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double angleOffset) {
    drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    rotate = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    this.canCoder = new CANCoder(canCoderID);
    encoder = rotate.getEncoder();
    encoder.setPosition(canCoder.getAbsolutePosition());
    anglePID.enableContinuousInput(0, 360);
    this.angleOffset = angleOffset;
    
  }
  public void setState(SwerveModuleState state){
      state = SwerveModuleState.optimize(state, new Rotation2d(canCoder.getAbsolutePosition()*Math.PI/180)); //try *Math.PI/180

        drive.set(state.speedMetersPerSecond/2.5);
        pidOut = anglePID.calculate(canCoder.getAbsolutePosition()+this.angleOffset, state.angle.getDegrees());
        //System.out.println(String.format(canCoder.getAbsolutePosition() + "-" + state.angle.getDegrees() + "-" + pidOut));
        rotate.set(pidOut/8);
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
