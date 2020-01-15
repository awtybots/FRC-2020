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
    public Vector3 setMagnitude(double mag) {
        double factor = mag/getMagnitude();
        x *= factor;
        y *= factor;
        z *= factor;
        return this;
    }
    public Vector3 normalize() {
        setMagnitude(1);
        return this;
    }
    public Vector3 setZ(double z) {
        this.z = z;
        return this;
    }
    public Vector3 rotateZ(double deg) {
        double s = Math.sin(Math.toRadians(deg));
        double c = Math.cos(Math.toRadians(deg));

        double x2 = x * c - y * s;
        double y2 = x * s + y * c;

        this.x = x2;
        this.y = y2;

        return this;
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
    @Override
    public Vector3 clone() {
        return new Vector3(x, y, z);
    }

}