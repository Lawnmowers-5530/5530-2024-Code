package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherAngle extends SubsystemBase {
    CANSparkMax motor;
    RelativeEncoder encoder;
    RelativeEncoder encoder2;
    PIDController pid;

    public LauncherAngle(int motorPort) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(false);
        encoder = motor.getAlternateEncoder(8192);
        encoder2 = motor.getEncoder();

        encoder.setPosition(0);
        encoder2.setPosition(0);

        pid = new PIDController(0.001, 0, 0);
        encoder.setPositionConversionFactor(8192);
        encoder2.setPositionConversionFactor(42);
        SmartDashboard.putNumber("Launcher Angle Subsystem Initialized: Conversion rate: ", encoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Launcher Angle Subsystem Initialized: Conversion rate 2: ", encoder2.getPositionConversionFactor());
    }

    public double getAngle() {
        return encoder.getPosition();
    }

    public void setAngle(double angle) {
        pid.setSetpoint(angle);
    }

    public void periodic() {
        motor.set(MathUtil.clamp(pid.calculate(encoder.getPosition()), -0.2, 0.35));
    }

    public void logAngle() {
        SmartDashboard.putNumber("Launcher Angle: ", encoder.getPosition());
        SmartDashboard.putNumber("Angle 2", encoder2.getPosition());
    }
}
