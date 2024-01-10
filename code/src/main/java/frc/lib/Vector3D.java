// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
public class Vector3D {
  private double[] vector;

  private double vx;
  private double vy;
  private double vz;

  public Vector3D(double vx, double vy, double vz, boolean isPolar){
      if(isPolar){

          double[] vector = VectorOperator.toCartesian3D(new double[]{vx, vy, vz});

          this.vector = vector;
          this.vx = vector[0];
          this.vy = vector[1];
          this.vz = vector[2];

      }else{
          this.vx = vx;
          this.vy = vy;
          this.vz = vz;

          this.vector = new double[]{vx, vy, vz};
      }
  }

  public double[] getVectorAsDouble(boolean polar){
      if(polar){
          return VectorOperator.toPolar3D(this);
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
  public double getvZ(){
      return this.vz;
  }
  public double getR(){
      return VectorOperator.toPolar3D(this)[0];
  }
  public double getTheta(){
      return VectorOperator.toPolar3D(this)[1];
  }
  public double getPhi(){
      return VectorOperator.toPolar3D(this)[2];
  }
  public void printVector(){
      System.out.println("["+this.vx+","+this.vy+","+this.vz+"]");
  }
}
