package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class LauncherAngle extends SubsystemBase implements Loggable{
    CANSparkMax motor;
    SparkAbsoluteEncoder encoder;

    PIDController upPid;

    //ArmFeedforward feedforward;
    //SparkPIDController pid;
    @Log
    double sP;

    @Log
    String pidOutput = "a";

    @Log
    String feedOutput = "a";

    @Log
    String encoderPos = "a";

    public LauncherAngle(int motorPort, boolean reversed, double kP, double kI, double kD, double conversionFactor) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(reversed);

        upPid = new PIDController(0.018, 0.00, 0.007);
        SmartDashboard.putData("PIDUp", upPid);

        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(360);
        encoder.setZeroOffset(116);
        sP = 30;
        upPid.setSetpoint(sP);

        //feedforward = new ArmFeedforward(0.05, 0.12, 0.1, 0.0);
    }
    @Config
    public void setAngle(double angle) {
        this.sP = angle;
        upPid.setSetpoint(angle);
    }

    public void periodic() {
        double pidOut;
        pidOut = upPid.calculate(encoder.getPosition());
        //double feedOut = feedforward.calculate(encoder.getPosition(), encoder.getVelocity());

        //motor.set(MathUtil.clamp(output, -0.25, 0.25));

        encoderPos = Double.toString(encoder.getPosition());

        pidOutput = Double.toString(pidOut);
        //feedOutput = Double.toString(feedOut);

        //motor.set(pidOut + feedOut);
    }
}