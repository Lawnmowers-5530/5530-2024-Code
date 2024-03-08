package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LedController.fixedPalattePatternType;
import frc.robot.subsystems.LedController_MultiAccess.LedControllerProxy;

public class DistanceSensor extends SubsystemBase {
    Rev2mDistanceSensor sensor;
    LedControllerProxy proxy;

    public DistanceSensor(LedControllerProxy proxy) {
        this.proxy = proxy;
        sensor = new Rev2mDistanceSensor(Port.kOnboard);
        sensor.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
        this.setEnabled(true);
    }

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

    @Override
    public void periodic(){
        if(getDistance()<30){
        proxy.setPattern(fixedPalattePatternType.FireLarge, 0);
        }else{
            proxy.setPattern(fixedPalattePatternType.FireMedium, 0);
        }

    }
}