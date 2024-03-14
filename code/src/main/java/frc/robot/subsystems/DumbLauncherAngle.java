package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DumbLauncherAngle extends SubsystemBase {
    public enum Angle {
        UP, DOWN, RELAXED
    }

    CANSparkMax motor;
    Angle state;
    double power = 0.1;
    RelativeEncoder encoder;
    boolean relaxed = true;
    
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

    public Command ampAngleCommand() {
        return this.runOnce(
                () -> {
                    this.forceUp();
                });
    }

    public Command speakerAngleCommand() {
        return this.runOnce(
                () -> {
                    this.forceDown();
                });
    }

    public boolean isAmpAngle() {
        return motor.get() > Constants.LauncherAngleConstants.ampPosition - Constants.LauncherAngleConstants.positionTolerance || motor.get() < Constants.LauncherAngleConstants.ampPosition + Constants.LauncherAngleConstants.positionTolerance;
    }
}