package frc.lib;

import frc.lib.module.launcher2024.subsystems.LauncherV2;
import frc.lib.module.launcherAngle2024.subsystems.LauncherAngle;

public class Shot {
    double speedfps;
    double rpm;

    double r;
    double phi;
    double theta;
    double thetaDeg;

    public Shot(double vx, double vy){
        this.speedfps = Math.sqrt(vx+vy);
    }
    public Shot(Vector3D vec){
        double vx = vec.getvX();
        double vy = vec.getvY();
        double vz = vec.getvZ();
        this.speedfps = Math.sqrt(Math.pow(vx,2)+Math.pow(vz,2));
        this.theta = Math.atan(vz/vx);
        this.thetaDeg = Math.toDegrees(this.theta);
        this.phi = Math.atan(vy/vx);
    }

    public double getThetaDeg(){
        return this.thetaDeg;
    }
    public double getSpeed(){
        return this.speedfps;
    }
    public double getPhiRad(){
        return this.phi;
    }

    public void Shoot(LauncherV2 launcher, LauncherAngle launcherAngle){
        double speed = this.getSpeed();
        double angle = this.getThetaDeg();

        double leftSpeed = speed;
        double rightSpeed = speed; //TODO

        launcher.setVelocity(leftSpeed, rightSpeed);
        launcherAngle.setAngle(angle);
    }

    @Override
    public String toString(){
        //show speed,theta,and phi
        return "Speed: " + this.speedfps + " ft/s,\nTheta: " + this.thetaDeg + " degrees,\nPhi: " + this.phi + " radians";
    }
}