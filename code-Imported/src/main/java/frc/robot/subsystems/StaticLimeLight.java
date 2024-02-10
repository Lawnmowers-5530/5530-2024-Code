// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.module.limelight.Limelight;
import io.github.oblarg.oblog.Loggable;
public class StaticLimeLight extends SubsystemBase implements Loggable{
  /** Creates a new LimeTest. */
  public StaticLimeLight() {}

  public static final Limelight limeLight = new Limelight();

  public static Pose2d getPose2DBlue() {
    return limeLight.getPose2DBlue();
}

public static Pose2d getPose2DRed() {
    return limeLight.getPose2DRed();
}

//get limelight tv value (whether or not there is a target)
public static boolean hasValidTargets() {
  return limeLight.hasValidTargets();
}
//get limelight tx value (horizontal offset from crosshair to target in degrees)
public static double getHorizontalOffset() {
  return limeLight.getHorizontalOffset();
}
//get limelight ty value (vertical offset from crosshair to target in degrees)
public static double getVerticalOffset() {
  return limeLight.getVerticalOffset();
}

public static BooleanSupplier validTargetSupp(){
  return () -> hasValidTargets();
}

public static boolean getValidTarget(){
  if(limeLight.getTargetArea()>=0.7){
    return true;
  }else{
    return false;
  }
}

public static double getTargetArea(){
  return limeLight.getTargetArea();
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
