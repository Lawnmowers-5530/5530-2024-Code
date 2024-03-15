package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax motor;
    public Intake(int motor, boolean reversed) {
        this.motor = new CANSparkMax(motor, CANSparkMax.MotorType.kBrushless);
        if (reversed == true) {
            this.motor.setInverted(true);
        }
    }
    public void run(double speed) {
        motor.set(speed);
    }

    public boolean isRunning() {
        return motor.get() != 0;
    }

    public Command intakeWheelCommand() {
        return this.run(() -> {
            this.run(Constants.IntakeConstants.intakeSpeed);
        });
    }

    public Command stopIntakeWheelCommand() {
        return this.runOnce(() -> {
            this.run(0);
        });
    }

    public Command ejectCommand() {
        return this.run(() -> {
            this.run(Constants.ejectSpeed);
        });
    }
}