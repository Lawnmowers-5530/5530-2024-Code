package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class LauncherV2 extends Launcher {
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder2;
    RelativeEncoder rightEncoder2;
    SparkPIDController leftPIDController;
    SparkPIDController rightPIDController;
    
    public LauncherV2() {
        super(Constants.LauncherConstants.leftMotorPort, Constants.LauncherConstants.rightMotorPort, Constants.LauncherConstants.isReversed);
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftPIDController = leftMotor.getPIDController();
        rightPIDController = rightMotor.getPIDController();
        leftEncoder2 = leftMotor.getAlternateEncoder(8192);
        rightEncoder2 = rightMotor.getAlternateEncoder(8192);
        leftPIDController.setP(Constants.LauncherConstants.kP);
        leftPIDController.setI(Constants.LauncherConstants.kI);
        leftPIDController.setD(Constants.LauncherConstants.kD);
        leftPIDController.setFF(Constants.LauncherConstants.kF);

        rightPIDController.setP(Constants.LauncherConstants.kP);
        rightPIDController.setI(Constants.LauncherConstants.kI);
        rightPIDController.setD(Constants.LauncherConstants.kD);
        rightPIDController.setFF(Constants.LauncherConstants.kF);
    }
public void setVelocity(double left, double right) {
        leftPIDController.setReference(left, CANSparkBase.ControlType.kVelocity);
        rightPIDController.setReference(right, CANSparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber("Left Target", left);
        SmartDashboard.putNumber("Right Target", right);
        SmartDashboard.putNumber("left velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("right velocity", rightEncoder.getVelocity());
    }
    
    @Override
    public void reset() {
        leftPIDController.setReference(0, CANSparkBase.ControlType.kDutyCycle);
        rightPIDController.setReference(0, CANSparkBase.ControlType.kDutyCycle);
    }

    public void logEncoder() {
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Encoder 2", leftEncoder2.getPosition());
        SmartDashboard.putNumber("Right Encoder 2", rightEncoder2.getPosition());
    }
}
