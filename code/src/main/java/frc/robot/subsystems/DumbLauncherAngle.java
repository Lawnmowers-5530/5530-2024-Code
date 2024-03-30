package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DumbLauncherAngleConstants.*;
import static frc.robot.Constants.LauncherAngleConstants.*;
import static frc.robot.Constants.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DumbLauncherAngle extends SubsystemBase implements Loggable {
    public enum Angle {
        UP, DOWN, RELAXED
    }

    CANSparkMax motor;
    Angle state;
    @Log
    double calculatedPower = 0;
    SparkAbsoluteEncoder encoder;

    @Log
    boolean isUp;
    @Log
    boolean isDown;

    public DumbLauncherAngle() {
        motor = new CANSparkMax(motorPort, CANSparkMax.MotorType.kBrushless);
        motor.setInverted(isReversed);
        sharedInit();
    }

    private void sharedInit() {
        this.encoder = this.motor.getAbsoluteEncoder();
        this.encoder.setPositionConversionFactor(conversionFactor);
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
                    calculatedPower = calculatedPower * atPositionMotorPowerMultiplier;
                }
                //System.out.println("set power up");
                break;
            }
            case DOWN: {
                calculatedPower = -power-0.025;
                if (isDown()) {
                    calculatedPower = calculatedPower * atPositionMotorPowerMultiplier;
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
        this.isUp = this.getEncoderMeasurement() < upPosition + positionTolerance;
        return this.isUp;
    }

    public boolean isDown() {
        this.isDown = this.getEncoderMeasurement() > downPosition - positionTolerance;
        return this.isDown;
    }
    @Log
    public double getTicks() {
        return encoder.getPosition();
    }

    @Override 
    public void periodic() {
        updateState();

        if (DEBUG_LOGGING) { //only update constantly when in debug mode
            isUp();
            isDown();
        }
    }
}