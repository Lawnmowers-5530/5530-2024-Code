package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ExternalIntake extends SubsystemBase implements Loggable {
    CANSparkMax pivotMotor;
    RelativeEncoder pivotEncoder;
    CANSparkMax rollerMotor;
    Position state = Position.UP;

    public enum Position {
        UP, DOWN, RELAXED
    }

    public ExternalIntake(int pivotMotorID, int rollerMotorID, boolean reversed) {
        pivotMotor = new CANSparkMax(pivotMotorID, CANSparkMax.MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        rollerMotor = new CANSparkMax(rollerMotorID, CANSparkMax.MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();

        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(Constants.ExternalIntakeConstants.pivotConversionFactor);

        if (reversed == true) {
            rollerMotor.setInverted(true);
        }
    }

    @Override
    public void periodic() {
        
        if (ready()) {
            pivotMotor.set(0);
            setPivotPosition(Position.RELAXED);
            return;
        }

        switch (state) {
            case UP:
                pivotMotor.set(Constants.ExternalIntakeConstants.pivotUpPower);
                break;
            case DOWN:
                pivotMotor.set(Constants.ExternalIntakeConstants.pivotDownPower);
                break;
            default:
                break;
        }
    }

    @Log
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    //check if the intake is in the correct position for both up or down
    @Log
    public boolean ready() {
        switch (state) {
            case UP:
                return pivotEncoder.getPosition() < Constants.ExternalIntakeConstants.pivotUpPosition;
            case DOWN:
                return pivotEncoder.getPosition() > Constants.ExternalIntakeConstants.pivotDownPosition;
            default:
                return false;
        }
    }
    
    public Position getState() {
        return state;
    }

    @Log
    public String stateString() {
        switch (state) {
            case UP:
                return "UP";
            case DOWN:
                return "DOWN";
            default:
                return "Hmm....";
        }
    }

    public void setPivotPosition(Position position) {
        state = position;
    }

    public void run(double speed) {
        rollerMotor.set(speed);
    }

    //public boolean isRunning() {
    //    return rollerMotor.get() != 0;
    //}

    public Command setPivotCommand(Position position) {
        return this.run(() -> {setPivotPosition(position);});
    }

    public Command externalIntakeWheelCommand() {
        return this.run(() -> {
            this.run(Constants.ExternalIntakeConstants.rollerSpeed);//Constants.ExternalIntakeConstants.rollerSpeed
        });
    }

    public Command stopExternalIntakeWheelCommand() {
        return this.runOnce(() -> {
            this.run(0);
        });
    }

    public Command ejectCommand() {
        return this.run(() -> {
            this.run(Constants.ejectSpeed);
        });
    }
 
}