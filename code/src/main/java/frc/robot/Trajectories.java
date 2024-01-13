package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class Trajectories {
    static TrajectoryConfig config = new TrajectoryConfig(4, 2)
            .setKinematics(Constants.kinematics);

    static Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),

            List.of(new Translation2d(-0.5, 0)),
            
            new Pose2d(-1, 2, new Rotation2d(0)),
            config);
}
