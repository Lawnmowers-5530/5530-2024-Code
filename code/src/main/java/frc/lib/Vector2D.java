// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

import edu.wpi.first.math.Vector;

/** Add your docs here. */
public class Vector2D {
  private double[] vector;

  private double vx;
  private double vy;

  public Vector2D(double vx, double vy, boolean isPolar){
      if(isPolar){

          double[] vector = VectorOperator.toCartesian3D(new double[]{vx, vy});

          this.vector = vector;
          this.vx = vector[0];
          this.vy = vector[1];

      }else{
          this.vx = vx;
          this.vy = vy;

          this.vector = new double[]{vx, vy};
      }
  }

  public double[] getVectorAsDouble(boolean polar){
      if(polar){
          return VectorOperator.toPolar2D(this);
      }else{
          return this.vector;
      }
  }

  public double getvX(){
      return this.vx;
  }
  public double getvY(){
      return this.vy;
  }
  public double getR(){
      return VectorOperator.toPolar2D(this)[0];
  }
  public double getTheta(){
      return VectorOperator.toPolar2D(this)[1];
  }
  public void printVector(){
      System.out.println("["+this.vx+","+this.vy+"]");
  }

  public Vector2D getDifference(Vector2D v1, Vector2D v2) {
        Vector2D V = new Vector2D(v2.getvX() - v1.getvX(), v2.getvY() - v1.getvY(), false);
        return V;
  }
}