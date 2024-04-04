package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

import static frc.robot.Constants.LauncherConstants.*;

public class Launcher extends SubsystemBase implements Loggable {
    CANSparkMax leftMotor;
    CANSparkMax rightMotor;
    
    
    public Launcher() {
        leftMotor = new CANSparkMax(leftMotorPort, CANSparkMax.MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorPort, CANSparkMax.MotorType.kBrushless);
        leftMotor.setInverted(!isReversed);
        rightMotor.setInverted(isReversed);
    }

    public void setSpeed(double left, double right) {
        leftMotor.set(left);
        rightMotor.set(right);
    }


}
