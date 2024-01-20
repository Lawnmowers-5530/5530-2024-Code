// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** Add your docs here. */
public class VectorOperator {
  public static PIDController poseController = new PIDController(0.1, 0.02, 0.00);


  public static Vector3D add(Vector3D v1, Vector3D v2){
      double vx = v1.getvX() + v2.getvX();
      double vy = v1.getvY() + v2.getvY();
      double vz = v1.getvZ() + v2.getvZ();

      return new Vector3D(vx, vy, vz, false);

  }
  public static Vector3D subtract(Vector3D v1, Vector3D v2){
      double vx = v1.getvX() - v2.getvX();
      double vy = v1.getvY() - v2.getvY();
      double vz = v1.getvZ() - v2.getvZ();

      return new Vector3D(vx, vy, vz, false);

  }
  public static Vector3D subtract(Vector3D v1, Vector2D v2){
    double vx = v1.getvX() - v2.getvX();
    double vy = v1.getvY() - v2.getvY();
    double vz = v1.getvZ();

    return new Vector3D(vx, vy, vz, false);
}

  public static double vecLen3D(Vector3D vector){
      double vx = vector.getvX();
      double vy = vector.getvY();
      double vz = vector.getvZ();

      return Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2) + Math.pow(vz, 2));
  }

  public static double[] toPolar3D(Vector3D vector){
      double vx = vector.getvX();
      double vy = vector.getvY();
      double vz = vector.getvZ();

      double r = vecLen3D(vector);

      double theta = Math.acos(vz/r);

      double phi = Math.signum(vy) * Math.acos(vx/(Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2))));


      return new double[]{r, theta, phi};



  }
  public static double[] toPolar2D(Vector2D vector){
    double vx = vector.getvX();
    double vy = vector.getvY();

    double r = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));

    double theta = Math.atan(vy/vx);


    return new double[]{r, theta};



}
  public static double[] toCartesian3D(double[] vector){
      double r = vector[0];
      double theta = vector[1];
      double phi = vector[2];

      System.out.println(r);
      System.out.println(theta);
      System.out.println(phi);

      double vx = r * Math.sin(theta) * Math.cos(phi);
      double vy = r * Math.sin(theta) * Math.sin(phi);
      double vz = r * Math.cos(theta);

      return new double[]{vx, vy, vz};
  }

  public static double[] toCartesian2D(double r, double theta){
    double vx = r * Math.cos(theta);
    double vy = r * Math.sin(theta);

    return new double[]{vx, vy};
  }

  public static double[] convertToConventional3D(double[] vector){
      double r = vector[0];
      double originalTheta = vector[1];
      double originalPhi = vector[2];

      double theta = 90 - originalPhi;
      double phi = 90 - originalTheta;

      return new double[]{r, theta, phi};
  }

  public static Vector2D rotateVector2D(Vector2D vector, Rotation2d angle){
    return new Vector2D((vector.getvY()*angle.getSin())+(vector.getvX()*angle.getCos()), vector.getvY()*angle.getCos()-(vector.getvX()*angle.getSin()), false);
    }

  public static Vector2D fromPose(Pose2d currentPose, Pose2d targetPose){
    double x = poseController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
    double y = poseController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());

    return new Vector2D(x, y, false);
  }

}