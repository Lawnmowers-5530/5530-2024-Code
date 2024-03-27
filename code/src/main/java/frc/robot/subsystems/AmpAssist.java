package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AmpAssist extends SubsystemBase implements Loggable{
    private Servo servo;
    @Log
    double pos;
    
    public AmpAssist() {
        servo = new Servo(Constants.AmpAssistConstants.servoPort);  
    }

    public Command up() {
        return new InstantCommand(() -> {
            set(Constants.AmpAssistConstants.up);
        });
    }
    public Command down() {
        return new InstantCommand(() -> {
            set(Constants.AmpAssistConstants.down);
        });
    } 
    @Config
    public void set(double angle) {
        this.pos = angle;
    }
    

    public void periodic() {
        servo.set(pos);
    }

    //public void disabledPeriodic() {
    //    servo.set(pos);
    //}
}
