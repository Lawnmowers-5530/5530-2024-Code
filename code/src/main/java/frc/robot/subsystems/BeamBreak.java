package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak extends SubsystemBase implements Loggable{
    
    
private final DigitalInput beamBreak = new DigitalInput(1); 
    
    @Log
  public Boolean outerIntakeIsLoaded() {
    return !beamBreak.get();
  }
    
}
