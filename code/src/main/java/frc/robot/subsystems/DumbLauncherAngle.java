package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DumbLauncherAngle extends SubsystemBase implements Loggable {
    public enum Angle {
        UP, DOWN, RELAXED
    }

    CANSparkMax motor;
    Angle state;
    double power = 0.1;
    SparkAbsoluteEncoder encoder;
    @Log
    double ticks;

    public DumbLauncherAngle(int motorPort, boolean isReversed) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(isReversed);
        sharedInit();
    }

    public DumbLauncherAngle(int motorPort, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        this.power = power;
        sharedInit();
    }

    public DumbLauncherAngle(int motorPort, boolean isReversed, double power) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(isReversed);
        this.power = power;
        sharedInit();
    }

    private void sharedInit() {
        this.encoder = this.motor.getAbsoluteEncoder();
        this.encoder.setPositionConversionFactor(1);
        this.state = Angle.RELAXED;
    }

    public void setState(Angle state) {
        switch (state) {
            case UP: {
                motor.set(power);
                //System.out.println("set power up");
                break;
            }
            case DOWN: {
                motor.set(-power);
                //System.out.println("set power down");
                break;
            }
            case RELAXED: {
                motor.set(0);
                //System.out.println("relax power");
                break;
            }
        }
        this.state = state;
    }

    public Angle getState() {
        return state;
    }
    
    public double getEncoderMeasurement() {
        return encoder.getPosition() * Constants.LauncherAngleConstants.conversionFactor;
    }

    public Command ampAngleCommand() {
        return this.runOnce(
                () -> {
                    this.setState(Angle.UP);
                });
    }

    public Command speakerAngleCommand() {
        return this.runOnce(
                () -> {
                    this.setState(Angle.DOWN);
                });
    }

    public Command relaxCommand() {
        return this.runOnce(
                () -> {
                    this.setState(Angle.RELAXED);
                });
    }

    public boolean isAmpAngle() {
        return this.getEncoderMeasurement() < Constants.LauncherAngleConstants.ampPosition + Constants.LauncherAngleConstants.positionTolerance;
    }

    @Override 
    public void periodic() {
        ticks = this.getEncoderMeasurement();

        setState(state);
    }
}