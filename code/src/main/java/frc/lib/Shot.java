package frc.lib;

public class Shot {
    private double speedfps;

    private double phi;
    private double theta;

    public Shot(double vx, double vy){
        this.speedfps = Math.sqrt(vx+vy);
    }
    public Shot(Vector3D vec){
        double vx = vec.getvX();
        double vy = vec.getvY();
        double vz = vec.getvZ();
        this.speedfps = Math.sqrt(Math.pow(vx,2)+Math.pow(vz,2));
        this.theta = Math.atan(vz/vx);
        this.phi = Math.atan(vy/vx);
    }

    public double getThetaDeg(){
        return Math.toDegrees(this.theta);
    }
    public double getSpeed(){
        return this.speedfps;
    }
    public double getPhiDeg(){
        return Math.toDegrees(this.phi);
    }

    @Override
    public String toString(){
        //show speed,theta,and phi
        return "Speed: " + this.speedfps + " ft/s,\nTheta: " + this.getThetaDeg() + " degrees,\nPhi: " + this.getPhiDeg()+ " degrees";
    }
}