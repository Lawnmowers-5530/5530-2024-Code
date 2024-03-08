// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib;

/** Add your docs here. */
public class Vector3D {

  private double vx;
  private double vy;
  private double vz;

  public Vector3D(double vx, double vy, double vz){
          this.vx = vx;
          this.vy = vy;
          this.vz = vz;
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
  public void printVector(){
      System.out.println("["+this.vx+","+this.vy+","+this.vz+"]");
  } //prints the vector in the form [vx,vy,vz]

}
