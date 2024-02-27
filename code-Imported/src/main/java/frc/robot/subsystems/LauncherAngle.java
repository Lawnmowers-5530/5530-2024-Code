package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class LauncherAngle extends SubsystemBase implements Loggable{
    CANSparkMax motor;
    SparkAbsoluteEncoder encoder;
    PIDController pid;

    public LauncherAngle(int motorPort, boolean reversed, double kP, double kI, double kD, double conversionFactor) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(reversed);
        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        pid = new PIDController(kP, kI, kP);
        //pid = new PIDController(0.002, 0, 0.004);
        encoder.setPositionConversionFactor(conversionFactor);
        SmartDashboard.putNumber("launch angle pos read", encoder.getPosition());
        //pid.setSetpoint(encoder.getPosition());
        SmartDashboard.putNumber("Launcher Angle Subsystem Initialized: Conversion rate: ", encoder.getPositionConversionFactor());

    }

    public void setAngle(double angle) {
        pid.setSetpoint(angle);
    }

    public void periodic() {
        double output = pid.calculate(encoder.getPosition());
        SmartDashboard.putNumber("angle motor output", output);
        SmartDashboard.putNumber("current angle pos", encoder.getPosition());
        SmartDashboard.putNumber("targetpos", pid.getSetpoint());
        SmartDashboard.putNumber("velocity", encoder.getVelocity());
        output = MathUtil.clamp(output, -1, 1);
        //ratio is tuned to 0.01
        /*double multiply_constant = pid.getSetpoint() * createInfo.constantRatio;

        SmartDashboard.putNumber("idk", multiply_constant);
        double deadband = MathUtil.clamp(multiply_constant * createInfo.constantRatio, createInfo.constantMin, createInfo.constantMax);
        SmartDashboard.putNumber("deadband", deadband);


        if (output > 0 ) {
            output = output + deadband;
        } else if (output < 0) {
            output = output - deadband;
        }
        //deadband ratio was 1.5
        if (Math.abs(output) < deadband * createInfo.deadbandRatio) {
            output = 0;
        }
        
        /*boolean negative = false;
        if (output < 0) {
            negative = true;
        } else if (output > 0) {
            negative = false;
        } else {
            negative = false;
        }

        output = Math.abs(output);

        if (output < 0.15 && encoder.getVelocity() < 1) {
            output = 0.15;
        }
        
        if (negative) {
            output = -output;
        }*/
        SmartDashboard.putNumber("Final output", output);   
        motor.set(output);
    }

    public void logAngle() {
        SmartDashboard.putNumber("Launcher Angle: ", encoder.getPosition());
    }
}