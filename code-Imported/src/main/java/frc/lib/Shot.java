package frc.lib;

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
}