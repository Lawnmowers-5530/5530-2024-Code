package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DumbLauncherAngle extends SubsystemBase {
    CANSparkMax motor;
    boolean relaxed = true;
    double power = 0.1;
    RelativeEncoder encoder;

    public DumbLauncherAngle(int motorPort, boolean isReversed) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        motor.setInverted(isReversed);
    }

    public DumbLauncherAngle(int motorPort, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        this.power = power;
    }

    public DumbLauncherAngle(int motorPort, boolean isReversed, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        motor.setInverted(isReversed);
        this.power = power;
    }

    public void forceDown() {
        motor.set(-power);
        relaxed = false;
    }

    public void forceUp() {
        motor.set(power);
        relaxed = false;
    }

    public void relax() {
        motor.set(0);
        relaxed = true;
    }

    public boolean getRelaxed() {
        return relaxed;
    }
    
    public double getEncoderMeasurement() {
        return encoder.getPosition();
    }
}