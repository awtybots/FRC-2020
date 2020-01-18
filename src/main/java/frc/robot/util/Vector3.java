package frc.robot.util;

import java.util.function.BiFunction;
import java.util.function.Function;

public class Vector3 {

    public double x;
    public double y;
    public double z;


    public Vector3() {
        this(0);
    }
    public Vector3(double n) {
        this(n, n, n);
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
        return applyFunction(n -> n*factor);
    }
    public Vector3 normalize() {
        setMagnitude(1);
        return this;
    }
    public Vector3 invert() {
        return applyFunction(n -> -n);
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
        return applyFunction((m, n) -> m + n, a, b);
    }
    public static Vector3 subtract(Vector3 a, Vector3 b) {
        return applyFunction((m, n) -> m - n, a, b);
    }
    public static Vector3 multiply(Vector3 a, Vector3 b) {
        return applyFunction((m, n) -> m * n, a, b);
    }
    public static Vector3 divide(Vector3 a, Vector3 b) {
        return applyFunction((m, n) -> m / n, a, b);
    }
    public static double dot(Vector3 a, Vector3 b) {
        Vector3 prod = multiply(a, b);
        return prod.x + prod.y + prod.z;
    }

    public Vector3 add(Vector3 b) {
        return add(this, b);
    }
    public Vector3 subtract(Vector3 b) {
        return subtract(this, b);
    }
    public Vector3 multiply(Vector3 b) {
        return multiply(this, b);
    }
    public Vector3 divide(Vector3 b) {
        return divide(this, b);
    }
    public double dot(Vector3 b) {
        return dot(this, b);
    }

    
    public Vector3 applyFunction(Function<Double, Double> function) {
        x = function.apply(x);
        y = function.apply(y);
        z = function.apply(z);
        return this;
    }
    public static Vector3 applyFunction(BiFunction<Double, Double, Double> function, Vector3 a, Vector3 b) {
        return new Vector3(
            function.apply(a.x, b.x),
            function.apply(a.y, b.y),
            function.apply(a.z, b.z)
        );
    }


    private double roundTenths(double n) {
        return Math.round(n * 10) / 10;
    }
    @Override
    public String toString() {
        Vector3 rounded = clone().applyFunction(this::roundTenths);
        return "( "+rounded.x+", "+rounded.y+", "+rounded.z+" )";
    }
    @Override
    public Vector3 clone() {
        return new Vector3(x, y, z);
    }
}