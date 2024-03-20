package frc.robot.subsystems;

import frc.lib.LimelightHelpers;

public class RingDetector {
    private String limelightName = "detector";

    public String getClassID(){
        return LimelightHelpers.getNeuralClassID(limelightName);
    }

    private LimelightHelpers.LimelightResults getResults(){
        return LimelightHelpers.getLatestResults(limelightName);
    }
    
    //private double getConfidence(LimelightHelpers.LimelightResults results){
    //    return results.targetingResults.targets_Detector.
//
    //}
}
