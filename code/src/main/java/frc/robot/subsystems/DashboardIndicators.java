package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DashboardIndicators extends SubsystemBase{
    private DistanceSensor distanceSensor;
    public DashboardIndicators(DistanceSensor distanceSensor){
        this.distanceSensor = distanceSensor;
    }
    public boolean isLoaded(){
        return distanceSensor.getDistance() < 15;
    }

    public void periodic() {
        SmartDashboard.putBoolean("isLoaded", isLoaded());
    }


}
