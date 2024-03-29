package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DistanceSensorMXP extends SubsystemBase implements Loggable{
    @Log
    double pos;
    Rev2mDistanceSensor sensor;

    public DistanceSensorMXP() {
        sensor = new Rev2mDistanceSensor(Port.kMXP);
        sensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
        this.setEnabled(true);
        CommandScheduler.getInstance().registerSubsystem(this);
        Shuffleboard.getTab("DistanceSensorMXP").addNumber("distance", this::getDistance);
    }

    public double getDistance() { // returns distance in mm
        return sensor.getRange();
    }

    public void setEnabled(boolean enabled) {
        sensor.setEnabled(enabled);
        sensor.setAutomaticMode(enabled);
        System.out.println("distance sensor MXP enabled: " + enabled);
    }

    public boolean getEnabled() {
        return sensor.isEnabled();
    }

    public boolean checkBeamBreak( double cutoffDistance) {
        if (getDistance() < cutoffDistance) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic(){
        pos = getDistance();
        
    }
}