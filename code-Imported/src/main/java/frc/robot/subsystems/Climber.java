// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Climber extends SubsystemBase {
    CANSparkMax motor;
    RelativeEncoder encoder;

    @Log
    public double target = 0;
    @Log
    private double encoderPos = 0;

    public double deadband = 630;
    public double speed = 0.4;
    boolean runToPosMode = false;

    public Climber(int motorID, boolean reversed) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        motor.setInverted(reversed);
        encoder.setPosition(0);
        encoder.setPositionConversionFactor(42);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
        runToPosMode = false;
    }

    @Override
    public void periodic() {
        encoderPos = encoder.getPosition();
        if (runToPosMode) {
            if (Math.abs(encoderPos - target) > deadband) {
                if (target < encoderPos) {
                    motor.set(-speed);
                }
                if (target > encoderPos) {
                    motor.set(speed);
                }
            } else {
                motor.set(0);
            }
        }
    }

    public void setTarget(double target) {
        this.target = target;
        runToPosMode = true;
    }
}

// high: 18465
