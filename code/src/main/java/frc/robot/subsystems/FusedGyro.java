package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class FusedGyro {
    //return navx rotation2d fused with pgyro rotation2d
    public Rotation2d getFusedRot() {
        return Pgyro.getRot(); //TODO: add navx output to fuse
    }
}
