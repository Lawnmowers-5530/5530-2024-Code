package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import static frc.robot.Constants.LoaderConstants.*;

public class DistanceSensor extends SubsystemBase implements Loggable{
    Rev2mDistanceSensor sensor;

    public DistanceSensor() {
        sensor = new Rev2mDistanceSensor(Port.kMXP);
        sensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
        Shuffleboard.getTab("DistanceSensor").addNumber("distance", this::getDistance);
        this.setEnabled(true);
    }
    @Log
    public double getDistance() { // returns distance in mm
        return sensor.getRange();
    }

    public void setEnabled(boolean enabled) {
        sensor.setEnabled(enabled);
        sensor.setAutomaticMode(enabled);
        System.out.println("distance sensor enabled: " + enabled);
    }

    public boolean getEnabled() {
        return sensor.isEnabled();
    }

    public boolean isNotePresent() {
        if (getDistance() < loaderCutoffDistance) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isNoteNotPresent() {
        return !isNotePresent();
    }
}