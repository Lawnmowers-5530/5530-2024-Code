// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase implements Loggable {
  private CANSparkMax motor;
  private RelativeEncoder encoder;

  @Log
  private double position;

  public Climber() {
    motor = new CANSparkMax(Constants.ClimberConstants.motorPort, MotorType.kBrushless);
    motor.setInverted(Constants.ClimberConstants.isReversed);
    encoder = motor.getEncoder();
    encoder.setPosition(0);
  }

  public Command runRaw(DoubleSupplier speed) {
    return this.run(() -> {
      motor.set(speed.getAsDouble());
    });
  }

  public Command run(double speed) {
    return this.run(() -> {
      if (encoder.getPosition() < Constants.ClimberConstants.maxHeight
          && encoder.getPosition() > Constants.ClimberConstants.minHeight) {
        motor.set(speed); // was a -speed : removed for sense making but may need to be redid
      } else {
        motor.set(0);
      }
    });
  }

  public Command moveDownCommand() {
    return this.run(() -> {
      if (encoder.getPosition() > Constants.ClimberConstants.maxHeight) {
        motor.set(0);
      } else {
        motor.set(Constants.ClimberConstants.speed);
      }
    });
  }

  public Command moveUpCommand() {
    return this.run(() -> {
      if (encoder.getPosition() < Constants.ClimberConstants.minHeight) {
        motor.set(0);
      } else {
        motor.set(-Constants.ClimberConstants.speed);
      }
    });
  }

  @Override
  public void periodic() {
    position = encoder.getPosition();
  }
}
