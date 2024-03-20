package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;

public class ShotCalculator {
    private static Vector2D straightShot(Pose2d currentPose){
        double dist = getDistPose(currentPose);

        double vx = LauncherConstants.vxScaleFactor * Math.log(dist);
        double vy = (LauncherConstants.height * vx - (0.5 * -9.81 * Math.pow(dist, 2)) / vx) / dist;

        return new Vector2D(vx, vy, false);
    }
    public static Shot vecFinal(Pose2d currentPose, Vector2D robotVec){
        Vector2D strShot = straightShot(currentPose);
        double angleToTarget = angleToTarget(currentPose);
        
        double maxSpeed = LauncherConstants.maxSpeed; //max speed in fps of shooter
        Vector3D goalVector;

        double vx = strShot.getvX();
        double vz = strShot.getvY();
        

        double vxFinal = vx*Math.sin(angleToTarget + (Math.PI/2));
        double vyFinal = vx*Math.cos(angleToTarget + (Math.PI/2));
        double vzFinal = vz;

        goalVector = new Vector3D(vxFinal, vyFinal, vzFinal);

        Vector3D shotVector = VectorOperator.subtract(goalVector, robotVec);
        if(shotVector.getMagnitude() > maxSpeed){
            return null;
        }
        Shot shot = new Shot(shotVector);

        return shot;
    }

    private static double angleToTarget(Pose2d currentPose){
        return Math.atan2((currentPose.getX()-Constants.targetTranslation.getX()),(currentPose.getY()-Constants.targetTranslation.getY()))-(Math.PI/2);
    }

    private static double getDistPose(Pose2d currentPose){
        double dist = Constants.targetTranslation.getDistance(currentPose.getTranslation());
        return dist;
    }
}

//*starting with the shot: 3d vector with 3 components *//
//*add the robot vector to the 3d vector to get result vector */
//*adjust shot vector to match goal vector, shot vector is just goal - robot vector */
