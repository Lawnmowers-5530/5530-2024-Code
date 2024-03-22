package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class Launcher extends SubsystemBase implements Loggable {
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


}
