package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}