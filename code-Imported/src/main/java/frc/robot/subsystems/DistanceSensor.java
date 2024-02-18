package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase{
    Rev2mDistanceSensor sensor;

    public DistanceSensor() {
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
        sensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
    }
    
    public double getDistance() { //returns distance in mm
        return sensor.getRange();
    }

    public void setEnabled(boolean enabled) {
        sensor.setEnabled(enabled);
        sensor.setAutomaticMode(enabled);
    }

    public boolean getEnabled() {
        return sensor.isEnabled();
    }
}
