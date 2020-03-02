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

import java.util.HashMap;
import java.util.function.Consumer;

public class DriveTrainSubsystem extends SubsystemBase {

    // motors
    private static WPI_TalonFX motorL1 = new WPI_TalonFX(MotorIDs.DRIVE_L1);
    private static WPI_TalonFX motorL2 = new WPI_TalonFX(MotorIDs.DRIVE_L2);
    private static WPI_TalonFX motorR1 = new WPI_TalonFX(MotorIDs.DRIVE_R1);
    private static WPI_TalonFX motorR2 = new WPI_TalonFX(MotorIDs.DRIVE_R2);

    // PID data
    private HashMap<MotorGroup, Double> goalVelocity = new HashMap<>();
    private HashMap<MotorGroup, Double> lastVelocityError = new HashMap<>();
    private HashMap<MotorGroup, Double> integralError = new HashMap<>();

    // NavX
    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    public DriveTrainSubsystem() {
        for(WPI_TalonFX motor : ALL.getMotors()) {
            motor.configFactoryDefault();
            motor.setNeutralMode(BRAKE_MODE);
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

            if(DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
                motor.configOpenloopRamp(1.0 / MAX_OUTPUT_ACCELERATION);
            }
        }

        for(WPI_TalonFX motor : RIGHT.getMotors()) {
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
            Consumer<MotorGroup> motorControlFunction = (MOTOR_CONTROL_MODE == MotorControlMode.PID) ? this::drivePID : this::driveFeedforward;
            motorControlFunction.accept(LEFT);
            motorControlFunction.accept(RIGHT);
        }

        // System.out.println("total velocity: "+getWheelVelocity(ALL)); // TODO remove prints
        // System.out.println("total distance: "+getWheelDistance(ALL));
    }

    public void drivePID(MotorGroup motorGroup) {
        double currentVelocity = getWheelVelocity(motorGroup);
        double goalAcceleration = goalVelocity.getOrDefault(motorGroup, 0.0) - currentVelocity;
        double velocityError = DRIVE_MODE == DriveMode.RAMPED_VELOCITY
            ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
            : goalAcceleration;
        double accelerationError = (velocityError - lastVelocityError.getOrDefault(motorGroup, 0.0)) / PERIOD;
        lastVelocityError.put(motorGroup, velocityError);
        integralError.put(motorGroup, clamp(integralError.getOrDefault(motorGroup, 0.0) + (velocityError * PERIOD), -INTEGRAL_MAX / PID_I, INTEGRAL_MAX / PID_I));

        double P = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain PID_P", PID_P) : PID_P) * velocityError;
        double I = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain PID_I", PID_I) : PID_I) * integralError.get(motorGroup);
        double D = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain PID_D", PID_D) : PID_D) * accelerationError;

        double output = P + I + D;
        setMotorOutput(motorGroup, output);
    }

    public void driveFeedforward(MotorGroup motorGroup) {
        double currentVelocity = getWheelVelocity(motorGroup);
        double goalAcceleration = goalVelocity.getOrDefault(motorGroup, 0.0) - currentVelocity;
        double constrainedGoalAcceleration = DRIVE_MODE == DriveMode.RAMPED_VELOCITY
            ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
            : goalAcceleration;
        double constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

        double S = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_S", FF_S) : FF_S) * Math.signum(constrainedGoalVelocity);
        double V = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_V", FF_V) : FF_V) * constrainedGoalVelocity;
        double A = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_A", FF_A) : FF_A) * constrainedGoalAcceleration;

        double voltage = S + V + A;
        setMotorVoltage(motorGroup, voltage);
    }


    // DRIVE COMMAND FUNCTIONS

    private void setMotorOutput(MotorGroup group, double pct) {
        if(Math.abs(pct) < MIN_OUTPUT) pct = 0;
        pct = clamp(pct, -MAX_OUTPUT, MAX_OUTPUT);
        for(WPI_TalonFX motor : group.getMotors()) {
            motor.set(pct);
        }
    }

    public void setMotorOutput(double left, double right) {
        setMotorOutput(LEFT, left);
        setMotorOutput(RIGHT, right);

        // TODO remove logging
        System.out.println("Drive output left: " + motorL1.get());
        System.out.println("Drive output right: " + motorR1.get());
        SmartDashboard.putNumber("Drive output left", motorL1.get());
        SmartDashboard.putNumber("Drive output right", motorR1.get());
    }

    public void setMotorVoltage(MotorGroup group, double voltage) {
        setMotorOutput(group, voltage/12.0);
    }

    public void setGoalVelocity(double left, double right) {
        goalVelocity.put(LEFT, clamp(left, -MAX_VELOCITY, MAX_VELOCITY));
        goalVelocity.put(RIGHT, clamp(left, -MAX_VELOCITY, MAX_VELOCITY));
    }

    public void stop() {
        setGoalVelocity(0, 0);
        setMotorOutput(0, 0);
    }


    // SENSORS

    public void resetSensors() {
        for(WPI_TalonFX motor : ALL.getMotors()) {
            motor.setSelectedSensorPosition(0);
        }
        navX.reset();
    }

    public double getRotation() {
        return -navX.getAngle();
    }

    public double getWheelDistance(MotorGroup motorGroup) {
        double totalUnits = 0;
        for(WPI_TalonFX motor : motorGroup.getMotors()) {
            // System.out.println("distance: "+motor.getSelectedSensorPosition());
            totalUnits += motor.getSelectedSensorPosition();
        }
        double revolutions = totalUnits / 2048.0 / motorGroup.getMotors().length;
        return revolutions * WHEEL_CIRCUMFERENCE;
    }

    public double getWheelVelocity(MotorGroup motorGroup) {
        double totalUnits = 0;
        for(WPI_TalonFX motor : motorGroup.getMotors()) {
            // System.out.println("velocity: "+motor.getSelectedSensorVelocity());
            totalUnits += motor.getSelectedSensorVelocity();
        }
        double revolutions = totalUnits / 2048.0 * 10.0 / motorGroup.getMotors().length;
        return revolutions * WHEEL_CIRCUMFERENCE;
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

        private MotorGroup(WPI_TalonFX[] motorList) {
            this.motorList = motorList;
        }

        public WPI_TalonFX[] getMotors() {
            return motorList;
        }
    }
}
