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


    public static Vector3 add(Vector3 a, Vector3 b) {
        return new Vector3(a.x+b.x, a.y+b.y, a.z+b.z);
    }
    public static Vector3 subtract(Vector3 a, Vector3 b) {
        return new Vector3(a.x-b.x, a.y-b.y, a.z-b.z);
    }

    public Vector3 add(Vector3 b) {
        return Vector3.add(this, b);
    }
    public Vector3 subtract(Vector3 b) {
        return Vector3.subtract(this, b);
    }


    @Override
    public String toString() {
        return "( "+x+", "+y+", "+z+" )";
    }

}