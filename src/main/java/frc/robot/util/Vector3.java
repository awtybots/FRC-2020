package frc.robot.util;

import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  public Vector3(Pose2d pose) {
    this(pose.getTranslation().getX(), pose.getTranslation().getY(), 0);
  }

  public Vector3(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  public Vector3(Vector3 b) {
    set(b);
  }

  public double getMagnitude() {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
  }

  public Vector3 setMagnitude(double mag) {
    double factor = mag / getMagnitude();
    return applyFunction(n -> n * factor);
  }

  public Vector3 normalize() {
    setMagnitude(1);
    return this;
  }

  public Vector3 invert() {
    return applyFunction(n -> -n);
  }

  public Vector3 set(Vector3 b) {
    this.x = b.x;
    this.y = b.y;
    this.x = b.z;
    return this;
  }

  public Vector3 setX(double x) {
    this.x = x;
    return this;
  }

  public Vector3 setY(double y) {
    this.y = y;
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

  public double getZAngle() {
    return Math.floorMod((int) Math.toDegrees(Math.atan2(y, x)), 360);
  }

  public double dot(Vector3 b) {
    return VectorMath.dot(this, b);
  }

  public Vector3 applyFunction(Function<Double, Double> function) {
    x = function.apply(x);
    y = function.apply(y);
    z = function.apply(z);
    return this;
  }

  public Vector3 add(Vector3 b) {
    return set(VectorMath.add(this, b));
  }

  public Vector3 subtract(Vector3 b) {
    return set(VectorMath.subtract(this, b));
  }

  public Vector3 multiply(Vector3 b) {
    return set(VectorMath.multiply(this, b));
  }

  public Vector3 multiply(double b) {
    return set(VectorMath.multiply(this, new Vector3(b)));
  }

  public Vector3 divide(Vector3 b) {
    return set(VectorMath.divide(this, b));
  }

  public Vector3 divide(double b) {
    return set(VectorMath.divide(this, new Vector3(b)));
  }

  public Vector3 print(String name) {
    SmartDashboard.putString(name, toString());
    // System.out.println(String.format("Vector3 '%s': ", name) + toString());
    return this;
  }

  public Translation2d toTranslation2d() {
    return new Translation2d(x, y);
  }

  @Override
  public String toString() {
    Vector3 rounded = clone().applyFunction((n) -> ((double) Math.round(n * 10)) / 10);
    return "( " + rounded.x + ", " + rounded.y + ", " + rounded.z + " )";
  }

  @Override
  public Vector3 clone() {
    return new Vector3(x, y, z);
  }

  public static class VectorMath {
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

    public static Vector3 applyFunction(
        BiFunction<Double, Double, Double> function, Vector3 a, Vector3 b) {
      return new Vector3(
          function.apply(a.x, b.x), function.apply(a.y, b.y), function.apply(a.z, b.z));
    }
  }
}
