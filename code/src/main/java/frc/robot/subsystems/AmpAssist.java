package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class AmpAssist extends SubsystemBase implements Loggable{
    private Servo servo;
    @Log
    double pos;
    
    public AmpAssist(int servoPort) {
        servo = new Servo(servoPort);  
    }

    public Command up() {
        return new InstantCommand(() -> {
            set(0.9);
        });
    }
    public Command down() {
        return new InstantCommand(() -> {
            set(0);
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
