// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class VectorOperator {
  public static PIDController poseController = new PIDController(0.5, 0.02, 0.00);


  public static Vector2D add(Vector2D v1, Vector2D v2){
      double vx = v1.getvX() + v2.getvX();
      double vy = v1.getvY() + v2.getvY();

      return new Vector2D(vx, vy, false);

  }
  public static Vector2D subtract(Vector2D v1, Vector2D v2){
    double vx = v1.getvX() - v2.getvX();
    double vy = v1.getvY() - v2.getvY();

    return new Vector2D(vx, vy, false);
}
  public static Vector3D subtract(Vector3D v1, Vector2D v2){
    double vx = v1.getvX() - v2.getvX();
    double vy = v1.getvY() - v2.getvY();


    return new Vector3D(vx, vy, v1.getvZ());
}



  public static double[] toPolar2D(Vector2D vector){
    double vx = vector.getvX();
    double vy = vector.getvY();

    double r = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));

    double theta = Math.atan2(vy, vx);


    return new double[]{r, theta};



}

  public static double[] toCartesian2D(double r, double theta){
    double vx = r * Math.cos(theta);
    double vy = r * Math.sin(theta);

    return new double[]{vx, vy};
  }

  public static Vector2D rotateVector2D(Vector2D vector, Rotation2d angle){
    return new Vector2D((vector.getvY()*angle.getSin())+(vector.getvX()*angle.getCos()), vector.getvY()*angle.getCos()-(vector.getvX()*angle.getSin()), false);
    }

  public static Vector2D fromPose(Pose2d currentPose, Pose2d targetPose){
    double x = -poseController.calculate(currentPose.getTranslation().getX(), targetPose.getTranslation().getX());
    double y = -poseController.calculate(currentPose.getTranslation().getY(), targetPose.getTranslation().getY());

    return new Vector2D(x, y, false);
  }

}