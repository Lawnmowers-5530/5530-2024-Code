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
    final double power = 0.2;
    @Log
    double calculatedPower = 0;
    SparkAbsoluteEncoder encoder;
    @Log
    double ticks;

    @Log
    boolean isUp;
    @Log
    boolean isDown;

    public DumbLauncherAngle(int motorPort, boolean isReversed) {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(isReversed);
        sharedInit();
    }

    private void sharedInit() {
        this.encoder = this.motor.getAbsoluteEncoder();
        this.encoder.setPositionConversionFactor(Constants.LauncherAngleConstants.conversionFactor);
        setState(Angle.RELAXED);
    }

    public void setState(Angle state) {
        this.state = state;
    }

    public void updateState() {
        switch (this.state) {
            case UP: {
                calculatedPower = power;
                if (isUp()) {
                    calculatedPower = calculatedPower * 0.2;
                }
                //System.out.println("set power up");
                break;
            }
            case DOWN: {
                calculatedPower = -power-0.025;
                if (isDown()) {
                    calculatedPower = calculatedPower * 0.2;
                }
                //System.out.println("set power down");
                break;
            }
            case RELAXED: {
                calculatedPower = 0;
                //System.out.println("relax power");
                break;
            }
        }
        motor.set(calculatedPower);
    }

    public Angle getState() {
        return state;
    }
    
    public double getEncoderMeasurement() {
        return encoder.getPosition();
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

    public boolean isUp() {
        this.isUp = this.getEncoderMeasurement() < Constants.LauncherAngleConstants.upPosition + Constants.LauncherAngleConstants.positionTolerance;
        return this.isUp;
    }

    public boolean isDown() {
        this.isDown = this.getEncoderMeasurement() > Constants.LauncherAngleConstants.downPosition - Constants.LauncherAngleConstants.positionTolerance;
        return this.isDown;
    }

    @Override 
    public void periodic() {
        ticks = this.getEncoderMeasurement();
        updateState();
        isUp();
        isDown();

    }
}