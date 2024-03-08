package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {
    CANSparkMax motorLeft;
    CANSparkMax motorRight;
    public Loader(int motorLeft, int motorRight, boolean reversed) {
        this.motorLeft = new CANSparkMax(motorLeft, CANSparkMax.MotorType.kBrushless);
        this.motorRight = new CANSparkMax(motorRight, CANSparkMax.MotorType.kBrushless);
        if (reversed == true) {
            this.motorLeft.setInverted(true);
        }
    }

    public void run(double speed) {
        motorLeft.set(speed);
        motorRight.set(speed);
    }
}
