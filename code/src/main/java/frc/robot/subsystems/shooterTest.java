// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooterTest extends SubsystemBase {
  private CANSparkMax motor1 = new CANSparkMax(Constants.motor1id, MotorType.kBrushless);
  private CANSparkMax motor2 = new CANSparkMax(Constants.motor1id, MotorType.kBrushless);

  public void runShooter(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  /** Creates a new shooterTest. */
  public shooterTest() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
