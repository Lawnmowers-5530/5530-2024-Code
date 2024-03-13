package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DumbLauncherAngle extends SubsystemBase {
    CANSparkMax motor;
    boolean relaxed = true;
    double power = 0.1;

    public DumbLauncherAngle(int motorPort, boolean isReversed) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(isReversed);
    }

    public DumbLauncherAngle(int motorPort, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
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
}