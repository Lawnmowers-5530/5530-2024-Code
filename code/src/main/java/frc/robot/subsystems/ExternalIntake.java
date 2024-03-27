package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExternalIntake extends SubsystemBase {
    CANSparkMax pivotMotor;
    CANSparkMax rollerMotor;

    public ExternalIntake(int pivotMotor, int rollerMotor, boolean reversed) {
        this.pivotMotor = new CANSparkMax(pivotMotor, CANSparkMax.MotorType.kBrushless);
        this.rollerMotor = new CANSparkMax(rollerMotor, CANSparkMax.MotorType.kBrushless);
        if (reversed == true) {
            this.rollerMotor.setInverted(true);
        }
    }

    public void run(double speed) {
        rollerMotor.set(speed);
    }

    public boolean isRunning() {
        return rollerMotor.get() != 0;
    }

    public Command externalIntakeWheelCommand() {
        return this.run(() -> {
            this.run(Constants.ExternalIntakeConstants.rollerSpeed);
        });
    }

    public Command stopExternalIntakeWheelCommand() {
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