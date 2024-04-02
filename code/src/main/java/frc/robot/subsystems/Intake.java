package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.*;

public class Intake extends SubsystemBase {
    CANSparkMax motor;

    public Intake() {
        this.motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.motor.setInverted(isReversed);
    }
    public void run(double speed) {
        motor.set(speed);
    }

    public boolean isRunning() {
        return motor.get() != 0;
    }

    public Command intakeWheelCommand() {
        return this.run(() -> {
            this.run(intakeSpeed);
        });
    }

    public Command stopIntakeWheelCommand() {
        return this.runOnce(() -> {
            this.run(0);
        });
    }

    public Command ejectCommand() {
        return this.run(() -> {
            this.run(ejectSpeed);
        });
    }
}