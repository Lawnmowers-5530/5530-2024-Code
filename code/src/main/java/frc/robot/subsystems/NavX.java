package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class NavX {
    public static AHRS gyro = new AHRS(Port.kMXP);
    
    public static AHRS getGyro(){
        return gyro;
    }
    public static double getDeg(){
        return gyro.getYaw();
    }
    public static Rotation2d getRot() {
        return gyro.getRotation2d();
    }
    public static double getRad(){
        return getDeg()*Math.PI/180;
    }
    public static void zeroGyro(){
        gyro.zeroYaw();
    } 
    public static double getHdgDeg(){
        double a;
        if(getDeg()>0){
        a = Math.abs(getDeg()%360);
        }else{
        a = 360-Math.abs(getDeg()%360);
        }
        return a;
    }
    public static double getHdgRad(){
        double a;
        if(getRad()>0){
        a = Math.abs(getRad()%(Math.PI*2));
        }else{
        a = (Math.PI*2)-Math.abs(getRad()%(Math.PI*2));
        }
        return a;
    }
    public static double getPitch(){
        return gyro.getPitch();
    }
    public static double getRoll(){
        return gyro.getRoll();
    }

    

}
