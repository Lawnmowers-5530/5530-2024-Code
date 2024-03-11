// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climber extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  public Climber() {
    motor = new CANSparkMax(Constants.ClimberConstants.motorPort, MotorType.kBrushless);
    motor.setInverted(Constants.ClimberConstants.isReversed);
    encoder = motor.getEncoder();
    encoder.setPosition(0);
  }

  public void runRaw(double speed){
    motor.set(-speed);
  }

  public void run(double speed) {
    if(encoder.getPosition() < Constants.ClimberConstants.maxHeight && encoder.getPosition() > Constants.ClimberConstants.minHeight){
      motor.set(-speed);
    }else{
      motor.set(0);
    }
  }

  public void moveDown(){
    if (encoder.getPosition() > Constants.ClimberConstants.maxHeight){
      motor.set(0);
    } else {
      motor.set(Constants.ClimberConstants.speed);
    }
  }
  
  public void moveUp(){
    if (encoder.getPosition() < Constants.ClimberConstants.minHeight){
      motor.set(0);
    } else {
      motor.set(-Constants.ClimberConstants.speed);
    }
  }

  public void runLimited( double speed ) {
    if (encoder.getPosition() > Constants.ClimberConstants.maxHeight){
      motor.set(0);
    } else if (encoder.getPosition() < Constants.ClimberConstants.minHeight){
      motor.set(0);
    } else {
      motor.set(speed);
    }
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber position", encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
