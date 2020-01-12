/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public final class Constants {

    public final static NeutralMode BRAKE_MODE = NeutralMode.Coast;

    public final static class Ports {
        public static final int XBOX_CONTROLLER = 0;
    }

    public final static class MotorIDs {
        public static final int MOTOR_L1 = 1;
        public static final int MOTOR_L2 = 3;
        public static final int MOTOR_L3 = 4;

        public static final int MOTOR_R1 = 12;
        public static final int MOTOR_R2 = 13;
        public static final int MOTOR_R3 = 14;
    }

    public final static class PID {
        public static final PIDController DRIVE_MOTORS = new PIDController(1, 1, 1);
    }

    public final static class Controller {
        // which stick (left or right) to use for each control of arcade drive (speed and rotation)
        public static final Hand SPEED_HAND = Hand.kLeft;
        public static final Hand ROTATION_HAND = Hand.kRight;
    }
}