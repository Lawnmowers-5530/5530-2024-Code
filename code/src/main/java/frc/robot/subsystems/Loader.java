package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.LoaderConstants.*;

public class Loader extends SubsystemBase {
    CANSparkMax motorLeft;
    CANSparkMax motorRight;
    public Loader() {
        this.motorLeft = new CANSparkMax(leftMotorPort, CANSparkMax.MotorType.kBrushless);
        this.motorRight = new CANSparkMax(rightMotorPort, CANSparkMax.MotorType.kBrushless);
        this.motorLeft.setInverted(isReversed);
    }

    public void run(double speed) {
        motorLeft.set(speed);
        motorRight.set(speed);
    }
}
