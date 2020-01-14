package frc.robot.util;

public class Vector3 {

    public double x;
    public double y;
    public double z;

    public Vector3() {
        new Vector3(0, 0, 0);
    }

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getMagnitude() {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public void setMagnitude(double mag) {
        double factor = mag/getMagnitude();
        x *= factor;
        y *= factor;
        z *= factor;
    }

    public void normalize() {
        setMagnitude(1);
    }

}