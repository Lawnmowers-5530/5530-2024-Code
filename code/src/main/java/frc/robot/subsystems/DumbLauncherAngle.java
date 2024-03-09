package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.data.GlobalState;
import io.github.oblarg.oblog.annotations.Log;

public class DumbLauncherAngle extends SubsystemBase {
    public enum Angle {
        UP, DOWN, RELAXED
    }

    CANSparkMax motor;
    Angle state;
    double power = 0.1;
    RelativeEncoder encoder;
    @Log
    double ticks;

    public DumbLauncherAngle(int motorPort, boolean isReversed) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        motor.setInverted(isReversed);
    }

    public DumbLauncherAngle(int motorPort, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        this.power = power;
    }

    public DumbLauncherAngle(int motorPort, boolean isReversed, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.encoder = motor.getEncoder();
        motor.setInverted(isReversed);
        this.power = power;
    }

    public void setState(Angle state) {
        switch (state) {
            case UP: {
                motor.set(power);
            }
            case DOWN: {
                motor.set(-power);
            }
            case RELAXED: {
                motor.set(0);
            }
        }
        this.state = state;
    }

    public boolean getState() {
        return state;
    }
    
    public double getEncoderMeasurement() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        ticks = encoder.getPosition();

        if (ticks < Constants.LauncherAngleConstants.LOW_ANGLE_MEASUREMENT && state == Angle.DOWN) {
            GlobalState.armReady = true;
        } else if (ticks > Constants.LauncherAngleConstants.HIGH_ANGLE_MEASUREMENT && state == Angle.UP) {
            GlobalState.armReady = true;
        } else {
            GlobalState.armReady = false;
        }
    }
}