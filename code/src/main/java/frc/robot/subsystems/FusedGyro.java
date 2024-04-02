package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class FusedGyro {
    double pigeonUpTime;
    double pigeonUpTimeLast;

    double navUpTime;
    double navUpTimeLast;

    public FusedGyro() {}

    public Rotation2d getYaw(){
        return pigeonUpTime == pigeonUpTimeLast ? Pgyro.getRot() : NavX.getRot();
    }

}
