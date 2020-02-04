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
import frc.robot.commands.shooter.AutoShoot.AimMode;
import frc.robot.commands.shooter.AutoShoot.TrajectoryCalculationMode;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
import frc.robot.subsystems.DriveTrainSubsystem.MotorControlMode;
import frc.robot.util.TalonWrapper;
import frc.robot.util.Vector3;

public final class Constants {

    public final static class MotorIDs {
        public static final int DRIVE_L1 = 8;
        public static final int DRIVE_L2 = 10;
        public static final int DRIVE_L3 = 11;

        public static final int DRIVE_R1 = 7;
        public static final int DRIVE_R2 = 9;
        public static final int DRIVE_R3 = 12;

        public static final int INTAKE = 3; // TODO

        public static final int SHOOTER_FLYWHEEL = 1;
        public static final int SHOOTER_TURRET = 2;

        public static final int CONTROL_PANEL_SPINNER = 4; // TODO
    }

    public final static class DriveTrain {
        public final static TalonWrapper.MotorType MOTOR_TYPE = TalonWrapper.MotorType.TALON_SRX;
        public final static DriveMode DRIVE_MODE = DriveTrainSubsystem.DriveMode.DIRECT;
        public final static MotorControlMode MOTOR_CONTROL_MODE = MotorControlMode.PID;

        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;

        public final static double DEADZONE = 0.1;
        public final static double MIN_MOTOR_OUTPUT = 0.2;
        public final static double MAX_TELEOP_MOTOR_OUTPUT = 0.4;

        public final static double GEAR_RATIO = 1/8;
        public final static double WHEEL_CIRCUMFERENCE = 5 * Math.PI;
        public final static double ROBOT_CIRMCUMFERENCE = 100; // this the the distance (in inches) each wheel travels when the robot spins one time around its center

        public final static double MAX_VELOCITY = 36; // inches per second
        public final static double MAX_ACCELERATION = 6; // inches per second^2
        public final static double GOAL_TOLERANCE = 0.5; // how many inches away do we stop

        // PID
        public final static double PID_P = 0.02;
        public final static double PID_I = 0;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MIN = -0.5;
        public final static double INTEGRAL_MAX = 0.5;

        // FEEDFORWARD FORMULA: FF_S + (FF_V * velocity) + (FF_A * acceleration)
        public final static double FF_S = 1.3; // voltage required to move a wheel any amount
        public final static double FF_V = 0.12; // voltage required to sustain a wheel's speed moving 1 inch per second
        public final static double FF_A = 0.1; // voltage required to accelerate wheel at 1 inch per second per second
    }

    public final static class Intake {
        public final static double MOTOR_SPEED = 0.5;
    }

    public final static class Shooter {
        public final static TrajectoryCalculationMode TRAJECTORY_CALCULATION_MODE = TrajectoryCalculationMode.PRESET_TARGET; // TODO temp
        public final static Vector3 PRESET_TARGET_DISPLACEMENT = new Vector3(0, 60, 68);
        public final static AimMode AIM_MODE = AimMode.TURRET;
        public final static ShooterSubsystem.MotorControlMode MOTOR_CONTROL_MODE = ShooterSubsystem.MotorControlMode.PID;
        public final static TalonWrapper.MotorType FLYWHEEL_MOTOR_TYPE = TalonWrapper.MotorType.TALON_FX;

        public final static NeutralMode TURRET_BRAKE_MODE = NeutralMode.Brake;
        public final static NeutralMode FLYWHEEL_BRAKE_MODE = NeutralMode.Coast;

        public final static double SHOOTER_HEIGHT = 24; // TODO change to actual height
        public final static double SHOOTER_ANGLE = 45; // TODO change to actual angle

        public final static double GRAVITY = 32.2 * 12.0; // inches/second^2

        public final static double FLYWHEEL_RATIO = 12.0 / 34.0 * 84.0 / 22.0; // TODO
        public final static double TURRET_RATIO = 1.0/3.0; // TODO

        public final static double MAX_REVS_PER_SECOND = 5000.0 / 60.0; // TODO implement
        public final static double MAX_ACCELERATION = 300;

        public final static double FLYWHEEL_TELEOP_SPEED = 1200.0 / 60.0; // RPS when you shoot manually (only plebeians shoot manually)
        public final static double FLYWHEEL_BANG_BANG_SPEED = 0.5; // percentage power for bang bang when on
        public final static double FLYWHEEL_GOAL_VELOCITY_THRESHOLD = 0; // RPS threshold
        public final static int FLYWHEEL_GOAL_RPS_AVERAGE_COUNT = 10; // how many frames of RPS to get the average from

        public final static double TURRET_MIN_SPEED = 0.2; // %
        public final static double TURRET_MAX_SPEED = 0.5; // %
        public final static double TURRET_GOAL_ANGLE_THRESHOLD = 2; // degrees to be satisfied with result
        public final static double TURRET_ANGLE_SLOW_THRESHOLD = 6; // degrees to start slowing down
        public final static double TURRET_START_ANGLE = 0; // from 0 to 360 (code doesn't let turret angle go <0 or >360 to prevent rotating too far)

        public final static double FLYWHEEL_CIRCUMFERENCE = 4.0 * Math.PI;
        public final static double FLYWHEEL_SLIPPING_FACTOR = 0.9;

        // PID
        public final static double PID_MIN = 0.1;
        public final static double PID_MAX = 1;
        public final static double PID_P = 0.02;
        public final static double PID_I = 0.06;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MAX = 1.0;

        // FEEDFORWARD FORMULA: FF_S + (FF_V * velocity) + (FF_A * acceleration)
        public final static double FF_S = 1.8; // voltage required to move the flywheel any amount
        public final static double FF_V = 0.005; // voltage required to sustain the flywheel's speed moving at 1 rev per second
        public final static double FF_A = 0.005; // voltage required to accelerate wheel at 1 rev per second per second
    }

    public final static class Limelight {
        public final static double CAMERA_MOUNTING_ANGLE = 2;
        public final static double CAMERA_HEIGHT = 5.1;

        public final static double SHOOTER_VISION_HEIGHT = 44;//89.75 TODO;
        public final static double LOADING_STATION_VISION_HEIGHT = 16.5; // TODO no need for vision on loading station

        public final static double SHOOTER_VISION_HEIGHT_OFFSET = SHOOTER_VISION_HEIGHT - CAMERA_HEIGHT;
        public final static double LOADING_STATION_VISION_HEIGHT_OFFSET = LOADING_STATION_VISION_HEIGHT - CAMERA_HEIGHT; // TODO no need for vision on loading station
    }

    public final static class NavX {
        // all positions must be relative to the center of the middle blue alliance driver station wall and based on blue alliance objects
        // increasing X goes closer to the red driver stations
        // increasing Y goes closer to the red trench run
        // yaw 0 is facing the red alliance stations
        // increasing yaw turns counter clockwise

        public final static Vector3 POWER_PORT_POSITION = new Vector3(648, -67.34, 50);//98.25 TODO);
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
        public final static double COLOR_PASSES = 7;

        public final static double MOTOR_SPEED = 0.3;
        public final static double ENCODER_UNITS = 4096; // TODO switch to 2048 for Falcon500 encoders

        public final static NeutralMode BRAKE_MODE = NeutralMode.Brake;
    }

    public final static class Controller {
        public static final int PORT = 0;

        public static final Hand SPEED_HAND = Hand.kLeft; // which stick (left or right) to use for each control of arcade drive (speed and rotation)
        public static final Hand ROTATION_HAND = Hand.kRight;
    }
}
