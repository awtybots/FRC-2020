/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.util.Vector3;

public final class Constants {

    public final static class MotorIDs {
        public static final int DRIVE_L1 = 1;
        public static final int DRIVE_L2 = 3;
        public static final int DRIVE_L3 = 4;
        public static final int DRIVE_R1 = 12;
        public static final int DRIVE_R2 = 13;
        public static final int DRIVE_R3 = 14;

        public static final int INTAKE = 1;

        public static final int SHOOTER = 3;

        public static final int CONTROL_PANEL_SPINNER = 11;
    }

    public final static class DriveTrain {
        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;
        public final static FeedbackDevice MOTOR_FEEDBACK_DEVICE = FeedbackDevice.CTRE_MagEncoder_Relative;
        public final static double ENCODER_UNITS = 4096;

        public final static boolean TUNING_MODE = false;
        public final static double TEST_SPEED = 0.2;

        public final static double DEADZONE = 0.1;
        
        public final static double WHEEL_CIRCUMFERENCE = 5 * Math.PI;
        public final static double ROBOT_CIRMCUMFERENCE = 100; // this the the distance (in inches) each wheel travels when the robot spins one time around its center

        public final static double MAX_VELOCITY = 36; // inches per second
        public final static double MAX_ACCELERATION = 6; // inches per second per second
        public final static double GOAL_TOLERANCE = 0.5; // how many inches away do we stop

        // PID
        public final static double PID_P = 0.02;
        public final static double PID_I = 0;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MIN = -100;
        public final static double INTEGRAL_MAX = 100;

        // FEEDFORWARD FORMULA: FF_S + (FF_V * velocity) + (FF_A * acceleration)
        public final static double FF_S = 3.0; // voltage required to move a wheel any amount
        public final static double FF_V = 0.1; // voltage required to sustain a wheel's speed moving 1 inch per second
        public final static double FF_A = 0.1; // voltage required to accelerate wheel at 1 inch per second per second
    }

    public final static class Intake {
        public final static double MOTOR_SPEED = 0.5;
    }

    public final static class Shooter {
        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;
        public final static FeedbackDevice MOTOR_FEEDBACK_DEVICE = FeedbackDevice.CTRE_MagEncoder_Relative;
        public final static double ENCODER_UNITS = 4096;
        public final static double WHEEL_CIRCUMFERENCE = 4 * Math.PI;

        public final static double SHOOTER_HEIGHT = 24;
        public final static double SHOOTER_ANGLE = 45;

        public final static double MAX_ACCELERATION = 10;
        public final static double SHOOTER_TELEOP_SPEED = 60;
        public final static double GOAL_VELOCITY_THRESHOLD = 5;
        public final static double GOAL_ANGLE_THRESHOLD = 2;

        public final static double DRAG = 600;

        // PID
        public final static double PID_P = 0.02;
        public final static double PID_I = 0;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MIN = -100;
        public final static double INTEGRAL_MAX = 100;

        // FEEDFORWARD FORMULA: FF_S + (FF_V * velocity) + (FF_A * acceleration)
        public final static double FF_S = 3.0; // voltage required to move the flywheel any amount
        public final static double FF_V = 0.1; // voltage required to sustain the flywheel's speed moving at 1 rev per second
        public final static double FF_A = 0.1; // voltage required to accelerate wheel at 1 rev per second per second
    }

    public final static class Limelight {
		public final static double CAMERA_MOUNTING_ANGLE = 20;
        public final static double CAMERA_HEIGHT = 20;
        
        public final static double SHOOTER_VISION_HEIGHT = 95;
        public final static double LOADING_STATION_VISION_HEIGHT = 95;
        
        public final static double SHOOTER_VISION_HEIGHT_OFFSET = SHOOTER_VISION_HEIGHT - CAMERA_HEIGHT;
        public final static double LOADING_STATION_VISION_HEIGHT_OFFSET = LOADING_STATION_VISION_HEIGHT - CAMERA_HEIGHT;
    }

    public final static class NavX {
        public final static Vector3 POWER_PORT_POSITION = new Vector3(0, 48, 98.25);
    }

    public final static class ColorSensor {
        public final static I2C.Port PORT = I2C.Port.kOnboard;

        public final static double[] BLUE    = new double[]{.12, .41, .45}; // rgb[0-1]
        public final static double[] GREEN   = new double[]{.18, .55, .25};
        public final static double[] RED     = new double[]{.52, .34, .13};
        public final static double[] YELLOW  = new double[]{.31, .55, .12};

		public final static double VERIFY_COLOR_TIME = 0.03; // time (seconds) to verify color is really there (0.125 is 60 RPM)
    }

    public final static class ControlPanelSpinner {
        public final static double COLOR_PASSES = 7;

        public final static double MOTOR_SPEED = 0.3;
        public final static double ENCODER_UNITS = 4096;

        public final static NeutralMode BRAKE_MODE = NeutralMode.Brake;
    }

    public final static class Controller {
        public static final int PORT = 0;
        
        public static final Hand SPEED_HAND = Hand.kLeft; // which stick (left or right) to use for each control of arcade drive (speed and rotation)
        public static final Hand ROTATION_HAND = Hand.kRight;
    }
}