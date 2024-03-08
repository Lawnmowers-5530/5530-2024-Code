// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax motor;

  public Climber() {
    motor = new CANSparkMax(Constants.ClimberConstants.motorPort, MotorType.kBrushless);
    motor.setInverted(Constants.ClimberConstants.isReversed);
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void moveDown(){
    if (motor.getEncoder().getPosition() > Constants.ClimberConstants.maxHeight){
      motor.set(0);
    } else {
      motor.set(-Constants.ClimberConstants.speed);
    }
  }
  
  public void moveUp(){
    if (motor.getEncoder().getPosition() < Constants.ClimberConstants.minHeight){
      motor.set(0);
    } else {
      motor.set(Constants.ClimberConstants.speed);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
