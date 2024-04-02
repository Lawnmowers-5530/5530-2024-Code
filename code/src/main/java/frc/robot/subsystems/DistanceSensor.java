package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class DistanceSensor extends SubsystemBase implements Loggable{
    @Log
    double pos;
    Rev2mDistanceSensor sensor;
    

    public DistanceSensor() {
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
        sensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
        Shuffleboard.getTab("DistanceSensor").addNumber("distance", this::getDistance);
        this.setEnabled(true);
    }

    public double getDistance() { // returns distance in mm
        if (sensor.getRange() == -1) {
            return 400;
        } else {
            return sensor.getRange();
        }
    }

    public void setEnabled(boolean enabled) {
        sensor.setEnabled(enabled);
        sensor.setAutomaticMode(enabled);
        System.out.println("distance sensor enabled: " + enabled);
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
        pos = this.sensor.getRange();
    }
}