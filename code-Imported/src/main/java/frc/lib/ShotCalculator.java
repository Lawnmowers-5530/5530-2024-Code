package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ShotCalculator {
    public static Shot vecFinal(Vector2D robotVec, double dist, double angleToTarget){
        dist = Units.metersToFeet(dist);
        double shooterHeight = 20/12; //height of shooter
        double derivPosSubtracted = 1; //distance behind goal point to consider derivative from
        double maxSpeed = 30; //max speed in fps of shooter
        double goalDeriv = 0.1; //goal derivative by derivPos
        double vxStep = 0.05; //step of vx to achieve goal derivative
        double scaleFactor = 1; //scale factor for original vx value
        Vector3D goalVector;
        double height = (78/12)-shooterHeight; //height of target

        double vx = dist*scaleFactor;
        double vz = 0;

        double g = -9.81;

        double deriv;

        boolean optimized=false;
        while(!optimized){
            vz = (height*vx-(0.5*g*Math.pow(dist,2))/vx)/dist; //calc vz
            deriv = ((dist-derivPosSubtracted)*g + vx*vz)/Math.pow(vx, 2); //calc deriv at goal point
            if(deriv<goalDeriv){ //step scaleFactor to get closer to deriv
                scaleFactor+=vxStep;
                vx = dist*scaleFactor;
            }else{
                optimized=true;
            }
            if(Math.sqrt(vx+vz)>maxSpeed){ //return null if shot is not possible
                return null;
            }
        }
        

        double vxFinal = vx*Math.cos(angleToTarget);
        double vyFinal = vx*Math.sin(angleToTarget);
        double vzFinal = vz;

        goalVector = new Vector3D(vxFinal, vyFinal, vzFinal);
        Shot shot = new Shot(VectorOperator.subtract(goalVector, robotVec));

        return shot;
    }

    public static double angleToTarget(Pose2d currentPose){
        return Math.atan2((Constants.targetTranslation.getX()-currentPose.getX()),(-Constants.targetTranslation.getY()+currentPose.getY()))-(Math.PI/2);
    }
}

//*starting with the shot: 3d vector with 3 components *//
//*add the robot vector to the 3d vector to get result vector */
//*adjust shot vector to match goal vector, shot vector is just goal - robot vector */
