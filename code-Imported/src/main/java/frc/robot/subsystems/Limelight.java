package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private NetworkTable table;
    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    //get limelight tv value (whether or not there is a target)
    public boolean hasValidTargets() {
        return table.getEntry("tv").getDouble(0) == 1;
    }
    //get limelight tx value (horizontal offset from crosshair to target in degrees)
    public double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0);
    }
    //get limelight ty value (vertical offset from crosshair to target in degrees)
    public double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0);
    }
    //get limelight ta value (target area in percent of image)
    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }
    //get limelight tl value (pipline latency contribution in milliseconds)
    public double getPipelineLatency() {
        return table.getEntry("tl").getDouble(0);
    }
    //get limelight cl value (camera latency contribution in milliseconds)
    public double getCameraLatency() {
        return table.getEntry("tl").getDouble(0);
    }
    //get limelight tshort value (shortest sidelength of the bounding box in pixels)
    public double getShortestSideLength() {
        return table.getEntry("tshort").getDouble(0);
    }
    //get limelight tlong value (longest sidelength of the bounding box in pixels)
    public double getLongestSideLength() {
        return table.getEntry("tlong").getDouble(0);
    }
    //get limelight thor value (horizontal sidelength of the rough bounding box in pixels)
    public double getHorizontalSideLength() {
        return table.getEntry("thor").getDouble(0);
    }
    //get limelight tvert value (vertical sidelength of the rough bounding box in pixels)
    public double getVerticalSideLength() {
        return table.getEntry("tvert").getDouble(0);
    }
    //get limelight getpipe value (pipline index of the current camera)
    public double getPipelineID() {
        return table.getEntry("getpipe").getDouble(0);
    }
    //get limelight json dump of all values
    public String getJSONDump() {
        return table.getEntry("json").getString("{ \"error\": \"no data\" }");
    }
    //get limelight tclass value (class id of primary neural network result)
    public double getClassID() {
        return table.getEntry("tclass").getDouble(0);
    }
    //get limelight tc value (average HSV color of the crosshair region)
    public double getCrosshairColor() {
        return table.getEntry("tc").getDouble(0);
    }

    public Pose2d getPose2DBlue() {
        double[] botpose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[]{0,0,0,0,0,0,-1,-1});
        Rotation2d rotation = new Rotation2d(botpose[3],botpose[4]);
        return new Pose2d(botpose[0],botpose[1],rotation);
    }

    public Pose2d getPose2DRed() {
        double[] botpose = table.getEntry("botpose_wpired").getDoubleArray(new double[]{0,0,0,0,0,0,-1,-1});
        Rotation2d rotation = new Rotation2d(botpose[3],botpose[4]);
        return new Pose2d(botpose[0],botpose[1],rotation);
    }
    
    public String testBotpose(){
        return Double.toString(table.getEntry("botpose_wpiblue").getDouble(0));
    }

}
