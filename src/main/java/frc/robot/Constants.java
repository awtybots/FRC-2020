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
import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
import frc.robot.subsystems.DriveTrainSubsystem.MotorControlMode;

public final class Constants {

    public final static class MotorIDs {
        public static final int DRIVE_L1 = 1;
        public static final int DRIVE_L2 = 2;

        public static final int DRIVE_R1 = 3;
        public static final int DRIVE_R2 = 4;

        public static final int INTAKE = 9;

        public static final int INDEXER_L = 8;
        public static final int INDEXER_R = 7;

        public static final int TOWER_1 = 5;
        public static final int TOWER_2 = 6;

        public static final int FLYWHEEL = 10;

        public static final int CLIMB_1 = 11;
        public static final int CLIMB_2 = 12;

        public static final int CONTROL_PANEL_SPINNER = 13;
    }

    public final static class SolenoidChannels {
        public static final int INTAKE_FWD = 0;
        public static final int INTAKE_REV = 4;
    }

    public final static class DriveTrain {
        // OPTIONS
        public final static DriveMode DRIVE_MODE = DriveMode.RAMPED_PERCENT;
        public final static MotorControlMode MOTOR_CONTROL_MODE = MotorControlMode.PID;
        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;
        public static final Hand SPEED_HAND = Hand.kLeft;
        public static final Hand ROTATION_HAND = Hand.kRight;
        public final static boolean TUNING_MODE = false; // change to true if you want to tune PID / FF in SmartDashboard

        // LIMITS
        public final static double MIN_OUTPUT = 0;
        public final static double MAX_OUTPUT = 0.9;
        public final static double MAX_OUTPUT_ACCELERATION = 0.6;

        public final static double MAX_VELOCITY = 36; // inches per second
        public final static double MAX_ACCELERATION = 6; // inches per second^2

        // AUTON
        public final static double MAX_OUTPUT_AUTON = 0.5;
        public final static double MAX_VELOCITY_AUTON = 36;
        public final static double ROTATE_DEGREES_SLOW_THRESHOLD = 5;

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
    }

    public final static class Intake {
        public final static double MOTOR_SPEED = 0.6;
    }

    public final static class IndexerTower {
        public final static double INDEXER_L_MOTOR_SPEED = 0.4;
        public final static double INDEXER_R_MOTOR_SPEED = 0.6;
        public final static double TOWER_MOTOR_SPEED = 1.0;
    }

    public final static class Climb {
        public final static double MOTOR_SPEED = 0.7;
    }

    public final static class Shooter {
        // OPTIONS
        public final static NeutralMode FLYWHEEL_BRAKE_MODE = NeutralMode.Coast;
        public final static boolean TUNING_MODE = false; // change to true if you want to tune PID in SmartDashboard

        // SPEEDS
        public final static double FLYWHEEL_TELEOP_SPEED_1 = 4000.0 / 60.0; // RPS when you shoot manually (only plebeians shoot manually)
        public final static double FLYWHEEL_TELEOP_SPEED_2 = 5000.0 / 60.0; // 3 presets (buttons A, B, X)
        public final static double FLYWHEEL_TELEOP_SPEED_3 = 6000.0 / 60.0;

        // AUTO AIM
        public final static double MAX_AIMING_DRIVE_OUTPUT = 0.5;
        public final static double TARGET_ANGLE_THRESHOLD = 2; // degrees
        public final static double TARGET_ANGLE_SLOW_THRESHOLD = 10; // degrees

        // FLYWHEEL
        public final static double FLYWHEEL_RATIO = 12.0 / 36.0 * 72.0 / 22.0;
        public final static double FLYWHEEL_GOAL_VELOCITY_THRESHOLD = 10.0 / 60.0; // RPS threshold flywheel must be within to shoot balls
        public final static double FLYWHEEL_MAX_VELOCITY = 7000.0 / 60.0;
        public final static double FLYWHEEL_MIN_OUTPUT = 0;
        public final static double FLYWHEEL_MAX_OUTPUT = 0.9;

        // PID
        public final static double PID_P = 0.02;
        public final static double PID_I = 0.04;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MAX = 1.0;
    }

    public final static class Limelight {
        public final static double CAMERA_MOUNTING_ANGLE = 41;
    }

    public final static class ControlPanelSpinner {
        public final static NeutralMode BRAKE_MODE = NeutralMode.Brake;
        public final static double COLOR_PASSES = 7;
        public final static double MOTOR_SPEED = 0.3;

        public final static I2C.Port PORT = I2C.Port.kOnboard;

        public final static double[] BLUE    = new double[]{.12, .41, .45}; // rgb[0-1]
        public final static double[] GREEN   = new double[]{.18, .55, .25};
        public final static double[] RED     = new double[]{.52, .34, .13};
        public final static double[] YELLOW  = new double[]{.31, .55, .12};

        public final static double VERIFY_COLOR_TIME = 0.03; // time (seconds) to verify color is really there (0.125 is 60 RPM)
        public final static double MINIMUM_COLOR_CONFIDENCE = 0.95;
    }

    public final static class Controller {
        public static final int PORT_1 = 0;
        public static final int PORT_2 = 1;

        public final static double DEADZONE = 0.3;
    }
}
