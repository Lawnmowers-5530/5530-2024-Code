package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class LauncherAngle extends SubsystemBase implements Loggable{
    CANSparkMax motor;
    SparkAbsoluteEncoder encoder;

    PIDController pidController;

    ArmFeedforward feedforward;
    @Log
    double sP;

    @Log
    double output;

    @Log
    double encoderPos;

    public LauncherAngle(int motorPort, boolean reversed, double conversionFactor) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(reversed);

        pidController = new PIDController(0,0,0); //TODO: change these values
        pidController.setIZone(0.02);

        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(conversionFactor);
        encoder.setZeroOffset(116);
        sP = 30;

        feedforward = new ArmFeedforward(0,0.12,0,0); //0.05, -0.0175, 0.03, -0.225
    }

    @Config
    public void setAngle(double angle) {
        this.sP = angle;
        pidController.setSetpoint(angle);
    }

    public void setTolerance(double tolerance) {
        pidController.setTolerance(tolerance);
    }

    public double getError() {
        return pidController.getPositionError();
    }

    public boolean withinTolerance() {
        return pidController.atSetpoint();
    }

    public void periodic() {
        encoderPos = encoder.getPosition();
        double encoderVelocity = encoder.getVelocity();
        output = pidController.calculate(encoderPos) - feedforward.calculate(encoderPos * Math.PI * 2, encoderVelocity * Math.PI * 2);

        motor.set(output);
    }

        public double getEncoderMeasurement() {
        return encoder.getPosition() * Constants.LauncherAngleConstants.conversionFactor;
    }

    public Command ampAngleCommand() {
        return this.runOnce(
                () -> {
                    this.setAngle(Constants.LauncherAngleConstants.ampPosition);
                });
    }

    public Command speakerAngleCommand() {
        return this.runOnce(
                () -> {
                    this.setAngle(Constants.LauncherAngleConstants.speakerPosition);
                });
    }

    public Command setAngleCommand(double angle) {
        return this.runOnce(
                () -> {
                    this.setAngle(angle);
                });
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public boolean isAmpAngle() {
        return this.getEncoderMeasurement() < Constants.LauncherAngleConstants.ampPosition + Constants.LauncherAngleConstants.positionTolerance;
    }
    
    public boolean isSpeakerAngle() {
        return this.getEncoderMeasurement() > Constants.LauncherAngleConstants.speakerPosition - Constants.LauncherAngleConstants.positionTolerance;
    }
}