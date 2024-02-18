package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    static final double kP = 0.0001;
    static final double kI = 0.0001;
    static final double kD = 0.0001;
    int leftMotorPort;
    int rightMotorPort;
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    
    
    public Launcher(int leftMotorPort, int rightMotorPort, boolean reversed) {
        this.leftMotorPort = leftMotorPort;
        this.rightMotorPort = rightMotorPort;
        leftMotor = new CANSparkMax(leftMotorPort, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, CANSparkMax.MotorType.kBrushless);
        leftMotor.setInverted(!reversed);
        rightMotor.setInverted(reversed);
    }

    public void setSpeed(double left, double right) {
        leftMotor.set(left);
        rightMotor.set(right);
    }

    public void reset() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
