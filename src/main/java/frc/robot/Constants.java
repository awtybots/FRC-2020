/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public final class Constants {

    public final static class DriveTrain {
        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;
        public final static FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.CTRE_MagEncoder_Relative; // TODO add encoders

        public final static double MAX_ROBOT_VELOCITY = 36; // inches per second
        public final static double MAX_ROBOT_ACCELERATION = 18; // inches per second per second
        public final static ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(MAX_ROBOT_VELOCITY, MAX_ROBOT_ACCELERATION)); // TODO tune
        
        public final static double ENCODER_UNITS = 4096;
        public final static double WHEEL_CIRCUMFERENCE = 5 * Math.PI; // TODO tune
        public final static double ROBOT_CIRMCUMFERENCE = 100; // this the the distance (in inches) each wheel travels when the robot spins one time around its center TODO tune
        
        public final static double AUTON_DRIVE_SPEED = 0.75; // TODO test
        public final static double AUTON_ROTATE_SPEED = 0.75; // TODO test
    }

    public final static class MotorIDs {
        public static final int MOTOR_L1 = 1;
        public static final int MOTOR_L2 = 3;
        public static final int MOTOR_L3 = 4;

        public static final int MOTOR_R1 = 12;
        public static final int MOTOR_R2 = 13;
        public static final int MOTOR_R3 = 14;
    }

    public final static class Controller {
        public static final int PORT = 0;
        
        public static final Hand SPEED_HAND = Hand.kLeft; // which stick (left or right) to use for each control of arcade drive (speed and rotation)
        public static final Hand ROTATION_HAND = Hand.kRight;
    }
}