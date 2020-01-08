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
        public static final int XBOX_CONTROLLER = 1;

        public static final int MOTOR_L1 = 0;
        public static final int MOTOR_L2 = 0;
        public static final int MOTOR_L3 = 0;
        public static final int MOTOR_L4 = 0;

        public static final int MOTOR_R1 = 0;
        public static final int MOTOR_R2 = 0;
        public static final int MOTOR_R3 = 0;
        public static final int MOTOR_R4 = 0;
    }

    public final static class MotorIDs {
        public static final int MOTOR_L1 = 1;
        public static final int MOTOR_L2 = 2;
        public static final int MOTOR_L3 = 3;
        public static final int MOTOR_L4 = 4;

        public static final int MOTOR_R1 = 5;
        public static final int MOTOR_R2 = 6;
        public static final int MOTOR_R3 = 7;
        public static final int MOTOR_R4 = 8;
    }

    public final static class Controller {
        public static final Hand SPEED_HAND = Hand.kLeft;
        public static final Hand ROTATION_HAND = Hand.kRight;
    }
}