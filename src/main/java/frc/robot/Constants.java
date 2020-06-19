package frc.robot;

import frc.robot.commands.drive.TeleopDrive.DriveControls;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;
import frc.robot.subsystems.DrivetrainSubsystem.MotorControlMode;

public final class Constants {

  public static final double PERIOD = 0.02;

  public static final class DriveTrain {
    public static final boolean IS_TUNING =
        false; // enable PID/FeedForward tuning on SmartDashboard

    // Mappings/Preferences
    public static final MotorControlMode MOTOR_CONTROL_MODE =
        MotorControlMode.PID; // target velocity --> motor output
    public static final DriveMode DRIVE_MODE =
        DriveMode.RAMPED_PERCENT; // acceleration curve of motors
    public static final DriveControls DRIVE_CONTROLS =
        DriveControls.ARCADE_DRIVE; // stick input --> target veocity

    // Autonomous
    public static final double MAX_OUTPUT_AUTON = 0.5;
    public static final double MAX_VELOCITY_AUTON = 36;
    public static final double ROTATE_DEGREES_SLOW_THRESHOLD = 5;

    // Mechanical
    public static final double WHEEL_CIRCUMFERENCE = 6.375 * Math.PI;
    public static final double TRACK_WIDTH = 26.5; // inches between left wheels and right wheels
    public static final double GEAR_RATIO = (12.0 / 40.0) * (24.0 / 34.0);
  }

  public static final class Shooter {
    // Enable PID tuning
    public static final boolean IS_TUNING = false;

    // Manual Shooting speeds
    public static final double MANUAL_RPM_1 = 4000.0;
    public static final double MANUAL_RPM_2 = 4250.0;
    public static final double MANUAL_RPM_3 = 6000.0;

    // AutoAim Thresholds
    public static final double MAX_AIMING_DRIVE_OUTPUT = 0.3;
    public static final double TARGET_THRESHOLD_DEG = 1; // degrees
    public static final double TARGET_SLOW_THRESHOLD_DEG = 15; // degrees
  }

  public static final class Limelight {
    public static final double HORIZONTAL_SKEW = 1.75; // degrees
    public static final double MOUNTING_ANGLE = 41; // degrees
    public static final double BOOT_TIME = 45; // seconds
  }

  public static final class ControlPanelSpinner {
    public static final double MOTOR_SPEED = 0.3;

    public static final double[] BLUE = new double[] {.12, .41, .45}; // rgb[0-1]
    public static final double[] GREEN = new double[] {.18, .55, .25};
    public static final double[] RED = new double[] {.52, .34, .13};
    public static final double[] YELLOW = new double[] {.31, .55, .12};

    public static final double VERIFY_COLOR_SEC =
        0.03; // wait time to verify color (0.125s is 60 RPM)
    public static final double MIN_COLOR_CONFIDENCE =
        0.95; // Must be this sure before declaring color
  }
}
