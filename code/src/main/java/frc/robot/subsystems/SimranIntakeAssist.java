package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.*;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SimranIntakeAssist extends SubsystemBase implements Loggable {
    CANSparkMax pivotMotor;
    RelativeEncoder pivotEncoder;
    CANSparkMax rollerMotor;



    public SimranIntakeAssist(int pivotMotorID, int rollerMotorID, boolean reversed) {
        pivotMotor = new CANSparkMax(pivotMotorID, CANSparkMax.MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        rollerMotor = new CANSparkMax(rollerMotorID, CANSparkMax.MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();

        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0);
        pivotEncoder.setPositionConversionFactor(ExternalIntakeConstants.pivotConversionFactor);

        if (reversed == true) {
            rollerMotor.setInverted(true);
        }
    }

    @Override
    public void periodic() {
    }
    @Log
    public Boolean atBottom(){
        return pivotEncoder.getPosition() > Constants.ExternalIntakeConstants.pivotDownPosition;
        //return pivotEncoder.getPosition() < Constants.ExternalIntakeConstants.pivotUpPosition;
    }
    @Log
    public Boolean atUp(){
        return pivotEncoder.getPosition() < Constants.ExternalIntakeConstants.pivotUpPosition;
    }
    @Log
    public double pivotEncoder(){
       return pivotEncoder.getPosition();
    }
    

    public SequentialCommandGroup intakeDown(){
        SequentialCommandGroup command = new SequentialCommandGroup(
            new RunCommand(()->{
                pivotMotor.set(ExternalIntakeConstants.pivotDownPower);
               
            }).until(this::atBottom),
         
            new InstantCommand(()->{
                pivotMotor.set(0);
               
            })
            
            
        );
        command.addRequirements(this);
        return command;
    }

        public SequentialCommandGroup intakeUp(){
        SequentialCommandGroup command = new SequentialCommandGroup(
            new RunCommand(()->{
                pivotMotor.set(-.25);
               
            }).until(this::atUp),
      
            new InstantCommand(()->{
                pivotMotor.set(0);
               
            })
            
            
        );
           command.addRequirements(this);
        return command;
    }

    public ParallelCommandGroup downAndSpin(){
        return new ParallelCommandGroup(
            intakeDown(),
            new InstantCommand(()->{
                rollerMotor.set(ExternalIntakeConstants.rollerSpeed);
            })
        );
    }

        public ParallelCommandGroup upAndStop(){
        return new ParallelCommandGroup(
            intakeUp(),
            new InstantCommand(()->{
                rollerMotor.set(0);
            })
        );
    }

        public ParallelCommandGroup downAndEject(){
        return new ParallelCommandGroup(
            intakeDown(),
            new InstantCommand(()->{
                rollerMotor.set(-ExternalIntakeConstants.rollerSpeed);
            })
        );
    }
      
 
}