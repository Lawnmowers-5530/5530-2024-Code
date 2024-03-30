package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class LauncherV2 extends Launcher{
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;
    RelativeEncoder leftEncoder2;
    RelativeEncoder rightEncoder2;
    @Log
    double leftVelocityTarget = 0;
    @Log
    double rightVelocityTarget = 0;
    SparkPIDController leftPIDController;
    SparkPIDController rightPIDController;

    public LauncherV2() {
        super(Constants.LauncherConstants.leftMotorPort, Constants.LauncherConstants.rightMotorPort,
                Constants.LauncherConstants.isReversed);
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
        if (Constants.DEBUG_LOGGING) {
            logEncoder();
        }
    }

    @Config
    public void setVelocity(double left, double right) {
        leftPIDController.setReference(left, CANSparkBase.ControlType.kVelocity);
        rightPIDController.setReference(right, CANSparkBase.ControlType.kVelocity);
        this.leftVelocityTarget = left;
        this.rightVelocityTarget = right;
    }

    @Log
    public double getVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    public Command reset() {
        return this.runOnce(() -> {
            leftPIDController.setReference(0, CANSparkBase.ControlType.kDutyCycle);
            rightPIDController.setReference(0, CANSparkBase.ControlType.kDutyCycle);
        });
    }

    public void logEncoder() {
        Shuffleboard.getTab("LauncherV2").addNumber("Left Encoder", () -> leftEncoder.getPosition());
        Shuffleboard.getTab("LauncherV2").addNumber("Right Encoder", () -> rightEncoder.getPosition());
        Shuffleboard.getTab("LauncherV2").addNumber("Left Encoder 2", () -> leftEncoder2.getPosition());
        Shuffleboard.getTab("LauncherV2").addNumber("Right Encoder 2", () -> rightEncoder2.getPosition());
    }

    public Command stopLauncherCommand() {
        return this.reset().andThen(
                this.runOnce(() -> {
                    leftMotor.set(0);
                    rightMotor.set(0);
                }));
    }

    public Command runLauncherCommand(DoubleSupplier leftVelocitySupplier, DoubleSupplier rightVelocitySupplier) {
        return this.runOnce(
            () -> {
                this.setVelocity(leftVelocitySupplier.getAsDouble(), rightVelocitySupplier.getAsDouble());
            }
        );
    }

    public Command ampLauncherCommand() {
        return this.runLauncherCommand(
            () -> {
                    return Constants.LauncherConstants.LAUNCHER_LOW_REVS;
            },
            () -> {
                    return Constants.LauncherConstants.LAUNCHER_LOW_REVS
                                    / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
            });
    }

    public Command speakerLauncherCommand() {
        return this.runLauncherCommand(
            () -> {
                    return Constants.LauncherConstants.LAUNCHER_HIGH_REVS;
            },
            () -> {
                    return Constants.LauncherConstants.LAUNCHER_HIGH_REVS
                                    / (1 - Constants.LauncherConstants.LAUNCHER_SPEED_DIFF_PERCENT);
            });
    }



    public boolean isRunningIntake(){
        return leftMotor.get() < 0 && rightMotor.get() < 0;
    }

}
