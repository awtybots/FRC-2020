/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.I2C;
import frc.robot.commands.drive.TeleopDrive.DriveControls;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;
import frc.robot.subsystems.DrivetrainSubsystem.MotorControlMode;

public final class Constants {

    public final static class DriveTrain {

        public final static boolean IS_TUNING = false; // whether or not to tune PID/FeedForward on SmartDashboard

        // OPTIONS
        public final static MotorControlMode MOTOR_CONTROL_MODE = MotorControlMode.PID; // How the motors are controlled to reach target velocity
        public final static DriveMode DRIVE_MODE                = DriveMode.RAMPED_PERCENT; // the response curve of the motors
        public final static DriveControls DRIVE_CONTROLS        = DriveControls.ARCADE_DRIVE;

        // AUTON
        public final static double MAX_OUTPUT_AUTON = 0.5;
        public final static double MAX_VELOCITY_AUTON = 36;
        public final static double ROTATE_DEGREES_SLOW_THRESHOLD = 5;

        // VALUES
        public final static double WHEEL_CIRCUMFERENCE = 6.375 * Math.PI;
        public final static double TRACK_WIDTH         = 26.5; // inches between left wheels and right wheels
        public final static double GEAR_RATIO          = 12.0 / 40.0 * 24.0 / 34.0;

    }

    public final static class Shooter {
        // Enable PID tuning TODO this should be tuned before comp
        public final static boolean IS_TUNING = false;

        // Manual Shooting speeds
        public final static double FLYWHEEL_MANUAL_RPM_1 = 4000.0;
        public final static double FLYWHEEL_MANUAL_RPM_2 = 4250.0;
        public final static double FLYWHEEL_MANUAL_RPM_3 = 6000.0;

        // Auto Aiming thresholds
        public final static double MAX_AIMING_DRIVE_OUTPUT = 0.3;
        public final static double TARGET_ANGLE_THRESHOLD = 1; // degrees
        public final static double TARGET_ANGLE_SLOW_THRESHOLD = 15; // degrees
    }

    public final static class Limelight {
        public final static double CAMERA_X_OFFSET = 1.75;
        public final static double CAMERA_MOUNTING_ANGLE = 41;

        public final static double STARTUP_TIME = 45; // seconds
    }

    public final static class ControlPanelSpinner {
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
        public final static double STICK_DEADZONE = 0.2;
        public final static double TRIGGER_DEADZONE = 0.1;
    }
}
