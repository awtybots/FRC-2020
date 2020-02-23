/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.commands.drive.TeleopDrive.JoystickSmoothing;
import frc.robot.commands.shooter.AutoShoot.AutoShootMode;
import frc.robot.commands.shooter.AutoShoot.TrajectoryCalculationMode;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
import frc.robot.util.Vector3;

public final class Constants {

    public final static class MotorIDs {
        public static final int DRIVE_L1 = 1;
        public static final int DRIVE_L2 = 9;

        public static final int DRIVE_R1 = 10;
        public static final int DRIVE_R2 = 4;

        public static final int INTAKE = 3;

        public static final int INDEXER_L = 8;
        public static final int INDEXER_R = 2;
        public static final int TOWER = 7;

        public static final int SHOOTER_FLYWHEEL = 11;
        public static final int SHOOTER_TURRET = 6;

        public static final int CONTROL_PANEL_SPINNER = 5;
    }

    public final static class SolenoidChannels {
        public static final int INTAKE_FWD = 4;
        public static final int INTAKE_REV = 5;

        public static final int CLIMB_FWD = 6;
        public static final int CLIMB_REV = 7;
    }

    public final static class DriveTrain {
        // OPTIONS
        public final static DriveMode TELEOP_DRIVE_MODE = DriveMode.VELOCITY;
        public final static DriveMode AUTON_DRIVE_MODE = DriveMode.DIRECT;
        public final static JoystickSmoothing JOYSTICK_SMOOTHING = JoystickSmoothing.NONE; // TODO change to square
        public final static DriveTrainSubsystem.MotorControlMode MOTOR_CONTROL_MODE = DriveTrainSubsystem.MotorControlMode.PID;
        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;
        public static final Hand SPEED_HAND = Hand.kLeft;
        public static final Hand ROTATION_HAND = Hand.kRight;
        public final static boolean TUNING_MODE = false; // change to true if you want to tune PID / FF in SmartDashboard

        // LIMITS
        public final static double MIN_MOTOR_OUTPUT = 0.1;
        public final static double MAX_MOTOR_OUTPUT = 0.9;
        public final static double MAX_VELOCITY = 36; // inches per second
        public final static double MAX_ACCELERATION = 6; // inches per second^2

        // VALUES
        public final static double GEAR_RATIO = 12.0 / 40.0 * 24.0 / 34.0;
        public final static double WHEEL_CIRCUMFERENCE = 6.375 * Math.PI;
        public final static double TRACK_WIDTH = 26.5; // inches between left wheels and right wheels

        // PID
        public final static double PID_P = 0.02;
        public final static double PID_I = 0;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MAX = 1.0;

        // FEEDFORWARD
        public final static double FF_S = 1.3; // voltage required to move a wheel any amount
        public final static double FF_V = 0.12; // voltage required to sustain a wheel's speed moving 1 inch per second
        public final static double FF_A = 0.1; // voltage required to accelerate wheel at 1 inch per second per second

        // AUTON COMMANDS
        public final static double ROTATE_DEGREES_SLOW_THRESHOLD = 5; // degrees to start slowing down (only if DRIVE_MODE is TRAPEZOIDAL_VELOCITY)
        public final static double ROTATE_DEGREES_GOAL_TOLERANCE = 2;

        // RAMSETE
        public final static double RAM_B = 2.0; // these are values from WPILib for meters
        public final static double RAM_Z = 0.7;
    }

    public final static class Intake {
        public final static double MOTOR_SPEED = 0.6;
    }

    public final static class IndexerTower {
        public final static double INDEXER_L_MOTOR_SPEED = 0.4;
        public final static double INDEXER_R_MOTOR_SPEED = 0.6;
        public final static double TOWER_MOTOR_SPEED = 0.5;
    }

    public final static class Shooter {
        // OPTIONS
        public final static AutoShootMode AUTO_SHOOT_MODE = AutoShootMode.JUST_AIM_TURRET;
        public final static TrajectoryCalculationMode TRAJECTORY_CALCULATION_MODE = TrajectoryCalculationMode.VISION_ONLY;
        public final static Vector3 PRESET_TARGET_DISPLACEMENT = new Vector3(0, 100, 0);
        public final static ShooterSubsystem.MotorControlMode FLYWHEEL_MOTOR_CONTROL_MODE = ShooterSubsystem.MotorControlMode.PID;
        public final static NeutralMode TURRET_BRAKE_MODE = NeutralMode.Brake;
        public final static NeutralMode FLYWHEEL_BRAKE_MODE = NeutralMode.Coast;
        public final static boolean TUNING_MODE = false; // change to true if you want to tune PID / FF in SmartDashboard

        // SPEEDS
        public final static double FLYWHEEL_TELEOP_SPEED_1 = 4000.0 / 60.0; // RPS when you shoot manually (only plebeians shoot manually)
        public final static double FLYWHEEL_TELEOP_SPEED_2 = 5000.0 / 60.0; // 3 presets (buttons A, B, X)
        public final static double FLYWHEEL_TELEOP_SPEED_3 = 6000.0 / 60.0;

        // PHYSICS
        public final static double GRAVITY = 32.2 * 12.0; // inches/second^2
        public final static double SHOOTER_HEIGHT = 44; // height of ball when it leaves the shooter
        public final static double SHOOTER_ANGLE = 34.8; // angle of ball when it leaves the shooter

        // FLYWHEEL
        public final static double FLYWHEEL_RATIO = 12.0 / 36.0 * 72.0 / 22.0;
        public final static double FLYWHEEL_CIRCUMFERENCE = 4.0 * Math.PI;
        public final static double FLYWHEEL_SLIPPING_FACTOR = 1; // percent of energy not lost to friction
        public final static double FLYWHEEL_GOAL_VELOCITY_THRESHOLD = 10.0 / 60.0; // RPS threshold flywheel must be within to shoot balls
        public final static int FLYWHEEL_GOAL_RPS_AVERAGE_COUNT = 10; // how many frames of RPS to get the average from
        public final static double FLYWHEEL_MAX_VELOCITY = 5000.0 / 60.0;
        public final static double FLYWHEEL_MIN_OUTPUT = 0.1;
        public final static double FLYWHEEL_MAX_OUTPUT = 0.8;

        // TURRET
        public final static double TURRET_RATIO = 1.0;
        public final static double TURRET_MIN_SPEED = 0.2;
        public final static double TURRET_MAX_SPEED = 0.5;
        public final static double TURRET_MAX_ACCEL = 0.2; // max % to increase by every second
        public final static double TURRET_GOAL_ANGLE_THRESHOLD = 1; // degrees to be satisfied with result
        public final static double TURRET_ANGLE_SLOW_THRESHOLD = 5; // degrees to start slowing down
        public final static double TURRET_WRAP_AROUND_THRESHOLD = 6; // if we don't see a target, how far away from 360 should we start moving the other direction
        public final static double TURRET_START_ANGLE = 180; // from 0 to 360 (code doesn't let turret angle go <0 or >360 to prevent rotating too far)
        public final static double TURRET_CLIMB_ANGLE = 180; // angle the turret goes to for the climber to fit
        public final static double TURRET_CLIMB_ANGLE_TOLERANCE = 3;

        // PID
        public final static double PID_P = 0.02;
        public final static double PID_I = 0.04;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MAX = 1.0;

        // FEEDFORWARD
        public final static double FF_S = 1.8; // voltage required to move the flywheel any amount
        public final static double FF_V = 0.005; // voltage required to sustain the flywheel's speed moving at 1 rev per second
        public final static double FF_A = 0.005; // voltage required to accelerate wheel at 1 rev per second per second
    }

    public final static class Limelight {
        public final static double CAMERA_MOUNTING_ANGLE = 41;
        public final static double CAMERA_HEIGHT = 34;

        public final static double SHOOTER_VISION_HEIGHT = 89.75;
        public final static double LOADING_STATION_VISION_HEIGHT = 16.5;

        public final static double SHOOTER_VISION_HEIGHT_OFFSET = SHOOTER_VISION_HEIGHT - CAMERA_HEIGHT;
        public final static double LOADING_STATION_VISION_HEIGHT_OFFSET = LOADING_STATION_VISION_HEIGHT - CAMERA_HEIGHT;
    }

    public final static class NavX {
        // all positions must be relative to the center of the middle blue alliance driver station wall and based on blue alliance objects
        // increasing X goes closer to the red driver stations
        // increasing Y goes closer to the red trench run
        // yaw 0 is facing the red alliance stations
        // increasing yaw turns counter clockwise

        public final static Vector3 POWER_PORT_POSITION = new Vector3(648, -67.34, 98.25);
        public final static Vector3 LOADING_BAY_POSITION = new Vector3(0, -61.34, 29);
    }

    public final static class ColorSensor {
        public final static I2C.Port PORT = I2C.Port.kOnboard;

        public final static double[] BLUE    = new double[]{.12, .41, .45}; // rgb[0-1]
        public final static double[] GREEN   = new double[]{.18, .55, .25};
        public final static double[] RED     = new double[]{.52, .34, .13};
        public final static double[] YELLOW  = new double[]{.31, .55, .12};

        public final static double VERIFY_COLOR_TIME = 0.03; // time (seconds) to verify color is really there (0.125 is 60 RPM)
        public final static double MINIMUM_COLOR_CONFIDENCE = 0.95;
    }

    public final static class ControlPanelSpinner {
        public final static NeutralMode BRAKE_MODE = NeutralMode.Brake;
        public final static double COLOR_PASSES = 7;
        public final static double MOTOR_SPEED = 0.3;
    }

    public final static class Controller {
        public static final int PORT_1 = 0;
        public static final int PORT_2 = 1;

        public final static double DEADZONE = 0.1;
    }
}
