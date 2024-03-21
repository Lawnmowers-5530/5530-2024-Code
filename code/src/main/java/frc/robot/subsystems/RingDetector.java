package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.lib.LimelightHelpers;

public class RingDetector {
    private String limelightName = "detector";

    private PIDController xController = new PIDController(0.1, 0, 0);
    private PIDController yController = new PIDController(0.1, 0, 0);

    public String getClassID(){
        return LimelightHelpers.getNeuralClassID(limelightName);
    }

    /**
     * @return returns robot relative forward movement required
     */
    public double getXMovement(){
        return xController.calculate(LimelightHelpers.getTY(limelightName));
    }

    /**
     * @return returns robot relative side movement required
     */
    public double getYMovement(){
        return yController.calculate(LimelightHelpers.getTX(limelightName));
    }


    
    //private void logDetectorOutput(LimelightHelpers.LimelightResults results){
    //    LimelightResults resultsArray = LimelightHelpers.getLatestResults(limelightName);
    //    SmartDashboard.putNumber("detector output length", resultsArray.targetingResults.targets_Detector.length);
//
//
    //}
}
