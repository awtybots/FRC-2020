/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.MotorIDs;

import static frc.robot.Robot.*;
import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.subsystems.DriveTrainSubsystem.MotorGroup.*;

public class DriveTrainSubsystem extends SubsystemBase {

    // motors
    private static WPI_TalonFX motorL1 = new WPI_TalonFX(MotorIDs.DRIVE_L1);
    private static WPI_TalonFX motorL2 = new WPI_TalonFX(MotorIDs.DRIVE_L2);
    private static WPI_TalonFX motorR1 = new WPI_TalonFX(MotorIDs.DRIVE_R1);
    private static WPI_TalonFX motorR2 = new WPI_TalonFX(MotorIDs.DRIVE_R2);

    // NavX
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    public DriveTrainSubsystem() {
        for(WPI_TalonFX motor : ALL.motorList) {
            motor.configFactoryDefault();
            motor.setNeutralMode(BRAKE_MODE);
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

            if(DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
                motor.configOpenloopRamp(1.0 / MAX_OUTPUT_ACCELERATION);
            }
        }

        for(WPI_TalonFX motor : RIGHT.motorList) {
            motor.setInverted(TalonFXInvertType.Clockwise);
        }

        if(TUNING_MODE) {
            SmartDashboard.setDefaultNumber("DriveTrain PID_P", PID_P);
            SmartDashboard.setDefaultNumber("DriveTrain PID_I", PID_I);
            SmartDashboard.setDefaultNumber("DriveTrain PID_D", PID_D);

            SmartDashboard.setDefaultNumber("DriveTrain FF_S", FF_S);
            SmartDashboard.setDefaultNumber("DriveTrain FF_V", FF_V);
            SmartDashboard.setDefaultNumber("DriveTrain FF_A", FF_A);
        }
    }

    @Override
    public void periodic() {
        if(DRIVE_MODE == DriveMode.VELOCITY || DRIVE_MODE == DriveMode.RAMPED_VELOCITY) {
            if(MOTOR_CONTROL_MODE == MotorControlMode.PID) {
                LEFT.drivePID();
                RIGHT.drivePID();
            } else {
                LEFT.driveFeedforward();
                RIGHT.driveFeedforward();
            }
        }

        // System.out.println("total velocity: "+getWheelVelocity()); // TODO remove prints
        // System.out.println("total distance: "+getWheelDistance());
    }

    // DRIVE COMMAND FUNCTIONS

    public void setMotorOutput(double left, double right) {
        LEFT.setMotorOutput(left);
        RIGHT.setMotorOutput(right);

        // TODO remove logging
        SmartDashboard.putNumber("Drive output left", motorL1.get());
        SmartDashboard.putNumber("Drive output right", motorR1.get());
    }

    public void setGoalVelocity(double left, double right) {
        LEFT.setGoalVelocity(left);
        RIGHT.setGoalVelocity(right);
    }

    public void stop() {
        setGoalVelocity(0, 0);
        setMotorOutput(0, 0);
    }


    // SENSORS

    public void resetSensors() {
        for(WPI_TalonFX motor : ALL.motorList) {
            motor.setSelectedSensorPosition(0);
        }
        navX.reset();
    }

    public double getRotation() {
        return -navX.getAngle();
    }


    // ENUMS

    public enum MotorControlMode {
        PID, FEEDFORWARD;
    }

    public enum DriveMode {
        PERCENT,
        RAMPED_PERCENT,
        VELOCITY,
        RAMPED_VELOCITY;
    }

    public enum MotorGroup {
        LEFT(new WPI_TalonFX[]{ motorL1, motorL2 }),
        RIGHT(new WPI_TalonFX[]{ motorR1, motorR2 }),
        ALL(new WPI_TalonFX[]{ motorL1, motorL2, motorR1, motorR2 });

        private WPI_TalonFX[] motorList;

        public double goalVelocity = 0;
        public double integralError = 0;
        public double lastVelocityError = 0;

        private MotorGroup(WPI_TalonFX[] motorList) {
            this.motorList = motorList;
        }

        public double getWheelDistance() {
            return motorList[0].getSelectedSensorPosition() / 2048.0 * WHEEL_CIRCUMFERENCE;
        }
        public double getWheelVelocity() {
            return motorList[0].getSelectedSensorVelocity() / 2048.0 * 10.0 * WHEEL_CIRCUMFERENCE;
        }

        private void setGoalVelocity(double goalVelocity) {
            this.integralError = 0;
            this.lastVelocityError = 0;
            this.goalVelocity = clamp(goalVelocity, -MAX_VELOCITY, MAX_VELOCITY);
        }

        private void drivePID() {
            double currentVelocity = getWheelVelocity();
            double goalAcceleration = goalVelocity - currentVelocity;
            double velocityError = DRIVE_MODE == DriveMode.RAMPED_VELOCITY
                ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
                : goalAcceleration;
            double accelerationError = (velocityError - lastVelocityError) / PERIOD;
            lastVelocityError = velocityError;
            integralError = clamp(integralError + (velocityError * PERIOD), -INTEGRAL_MAX / PID_I, INTEGRAL_MAX / PID_I);

            double P = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain PID_P", PID_P) : PID_P) * velocityError;
            double I = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain PID_I", PID_I) : PID_I) * integralError;
            double D = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain PID_D", PID_D) : PID_D) * accelerationError;

            double output = P + I + D;
            setMotorOutput(output);
        }

        private void driveFeedforward() {
            double currentVelocity = getWheelVelocity();
            double goalAcceleration = goalVelocity - currentVelocity;
            double constrainedGoalAcceleration = DRIVE_MODE == DriveMode.RAMPED_VELOCITY
                ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
                : goalAcceleration;
            double constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

            double S = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_S", FF_S) : FF_S) * Math.signum(constrainedGoalVelocity);
            double V = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_V", FF_V) : FF_V) * constrainedGoalVelocity;
            double A = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_A", FF_A) : FF_A) * constrainedGoalAcceleration;

            double voltage = S + V + A;
            setMotorOutput(voltage/12.0);
        }

        private void setMotorOutput(double pct) {
            if(Math.abs(pct) < MIN_OUTPUT) pct = 0;
            pct = clamp(pct, -MAX_OUTPUT, MAX_OUTPUT);
            for(WPI_TalonFX motor : motorList) {
                motor.set(pct);
            }
        }
    }
}
