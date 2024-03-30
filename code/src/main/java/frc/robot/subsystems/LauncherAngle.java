package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class LauncherAngle extends SubsystemBase implements Loggable{
    CANSparkMax motor;
    SparkAbsoluteEncoder encoder;

    PIDController upPid;

    ArmFeedforward feedforward;
    //SparkPIDController pid;
    @Log
    double sP;

    @Log
    double pidOutput = 0;

    @Log
    double feedOutput = 0;

    @Log
    double encoderPos = 0;

    public LauncherAngle(int motorPort, boolean reversed, double kP, double kI, double kD, double conversionFactor) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(reversed);

        upPid = new PIDController(0.009, 0.005, 0.00);
        upPid.setIntegratorRange(-0.2, 0.2);
        upPid.setIZone(15);

        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);
        encoder.setZeroOffset(116);
        sP = 30;

        feedforward = new ArmFeedforward(0.07, -0.03, 0.035, -0.35); //0.05, -0.0175, 0.03, -0.225
    }

    @Config
    public void setAngle(double angle) {
        this.sP = angle;
        upPid.setSetpoint(angle);
    }

    public void setTolerance(double tolerance) {
        upPid.setTolerance(tolerance);
    }

    public double getError() {
        return upPid.getPositionError();
    }

    public boolean withinTolerance() {
        return upPid.atSetpoint();
    }

    public void periodic() {

        double pidOut;
        pidOut = upPid.calculate(encoder.getPosition(), sP);
        double feedOut = feedforward.calculate(encoder.getPosition(), MathUtil.applyDeadband(encoder.getVelocity(), 0.005));

        //motor.set(MathUtil.clamp(output, -0.25, 0.25));

        encoderPos = encoder.getPosition();

        pidOutput = pidOut;
        feedOutput = feedOut;

        //motor.set(feedOut + pidOut);
    }
}