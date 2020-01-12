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

public final class Constants {

    public final static class Values {
        public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;
        public final static FeedbackDevice DRIVE_MOTOR_FEEDBACK_DEVICE = FeedbackDevice.CTRE_MagEncoder_Relative;
        public final static double ENCODER_UNITS = 4096;
        public final static double AUTON_DRIVE_SPEED = 0.75;
        public final static double AUTON_TURN_SPEED = 0.75;
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