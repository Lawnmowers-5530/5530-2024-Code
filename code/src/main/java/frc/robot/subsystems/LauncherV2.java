package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.*;
import static frc.robot.Constants.LauncherConstants.*;


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
        super();
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
        leftPIDController = leftMotor.getPIDController();
        rightPIDController = rightMotor.getPIDController();
        leftEncoder2 = leftMotor.getAlternateEncoder(encoderCountsPerRev);
        rightEncoder2 = rightMotor.getAlternateEncoder(encoderCountsPerRev);
        leftPIDController.setP(kP);
        leftPIDController.setI(kI);
        leftPIDController.setD(kD);
        leftPIDController.setFF(kF);

        rightPIDController.setP(kP);
        rightPIDController.setI(kI);
        rightPIDController.setD(kD);
        rightPIDController.setFF(kF);
        if (debugLogging) {
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
                    return launcherLowRevs;
            },
            () -> {
                    return launcherLowRevs
                                    / (1 - launcherSpeedDiffPercent);
            });
    }

    public Command speakerLauncherCommand() {
        return this.runLauncherCommand(
            () -> {
                    return launcherHighRevs;
            },
            () -> {
                    return launcherHighRevs
                                    / (1 - launcherSpeedDiffPercent);
            });
    }

    public Command lobLauncherCommand() {
        return this.runLauncherCommand(
            () -> {
                    return launcherMedRevs;
            },
            () -> {
                    return launcherMedRevs
                                    / (1 - launcherMedRevs);
            });
    }



    public boolean isRunningIntake(){
        return leftMotor.get() < 0 && rightMotor.get() < 0;
    }

}
