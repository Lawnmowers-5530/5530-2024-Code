// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.module.limelight.Limelight;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class LimeTest extends SubsystemBase implements Loggable{

  public static Pose2d getLime(Limelight limelight){
    return limelight.getPose2DBlue();
  }
  
  /** Creates a new LimeTest. */
  public LimeTest() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
