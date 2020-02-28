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

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;
import frc.robot.util.Vector3;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Constants.NavX.*;
import static frc.robot.subsystems.DriveTrainSubsystem.MotorGroup.*;

import java.util.HashMap;
import java.util.function.Consumer;

public class DriveTrainSubsystem extends SubsystemBase {

    // this is the subsystem that interacts with the drivetrain motors

    private static WPI_TalonFX motorL1 = new WPI_TalonFX(MotorIDs.DRIVE_L1);
    private static WPI_TalonFX motorL2 = new WPI_TalonFX(MotorIDs.DRIVE_L2);

    private static WPI_TalonFX motorR1 = new WPI_TalonFX(MotorIDs.DRIVE_R1);
    private static WPI_TalonFX motorR2 = new WPI_TalonFX(MotorIDs.DRIVE_R2);

    private HashMap<MotorGroup, Double> goalVelocity = new HashMap<>();
    private HashMap<MotorGroup, Double> currentPercent = new HashMap<>();
    private HashMap<MotorGroup, Double> lastVelocityError = new HashMap<>();
    private HashMap<MotorGroup, Double> integralError = new HashMap<>();

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private DifferentialDriveOdometry odometry;
    private Vector3 lastPosition = new Vector3();
    private Vector3 position = new Vector3();

    private double initialAngle;

    private DriveMode CURRENT_DRIVE_MODE = DriveMode.PERCENT;
    private static double PERIOD = 0.02;


    // DRIVING

    public DriveTrainSubsystem() {
        PERIOD = Robot.getLoopTime();

        for(WPI_TalonFX motor : ALL.getMotors()) {
            motor.set(0); // start all motors at 0% speed to stop the blinking

            motor.configFactoryDefault(); // reset settings
            motor.setNeutralMode(BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // sets which encoder the motor is using
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
        if(CURRENT_DRIVE_MODE == DriveMode.VELOCITY || CURRENT_DRIVE_MODE == DriveMode.RAMPED_VELOCITY) {
            MOTOR_CONTROL_MODE.getMotorControlFunction(this).accept(LEFT);
            MOTOR_CONTROL_MODE.getMotorControlFunction(this).accept(RIGHT);
        } else if(CURRENT_DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
            driveRampedPercent(LEFT);
            driveRampedPercent(RIGHT);
        }

        if(odometry != null) {
            odometry.update(getRawRotation(), getWheelDistance(LEFT), getWheelDistance(RIGHT));
            lastPosition = position.clone();
            position = new Vector3(odometry.getPoseMeters()).print("Position");
            getVelocity().print("Velocity");
            SmartDashboard.putNumber("Rotation", getRotation());
        }

        // System.out.println("total velocity: "+getWheelVelocity(ALL)); // TODO remove prints
        // System.out.println("total distance: "+getWheelDistance(ALL));
    }

    private void drivePID(MotorGroup motorGroup) {
        double currentVelocity = getWheelVelocity(motorGroup);
        double goalAcceleration = goalVelocity.getOrDefault(motorGroup, 0.0) - currentVelocity;
        double velocityError = CURRENT_DRIVE_MODE == DriveMode.RAMPED_VELOCITY
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

    private void driveFeedforward(MotorGroup motorGroup) {
        double currentVelocity = getWheelVelocity(motorGroup);
        double goalAcceleration = goalVelocity.getOrDefault(motorGroup, 0.0) - currentVelocity;
        double constrainedGoalAcceleration = CURRENT_DRIVE_MODE == DriveMode.RAMPED_VELOCITY
            ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
            : goalAcceleration;
        double constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

        double S = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_S", FF_S) : FF_S) * Math.signum(constrainedGoalVelocity);
        double V = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_V", FF_V) : FF_V) * constrainedGoalVelocity;
        double A = (TUNING_MODE ? SmartDashboard.getNumber("DriveTrain FF_A", FF_A) : FF_A) * constrainedGoalAcceleration;

        double voltage = S + V + A;
        setMotorVoltage(motorGroup, voltage);
    }

    private void driveRampedPercent(MotorGroup motorGroup) {
        double goalPercent = goalVelocity.getOrDefault(motorGroup, 0.0);
        double rampedPercentAccel = clamp(goalPercent - currentPercent.getOrDefault(motorGroup, 0.0), -MAX_MOTOR_ACCEL*PERIOD, MAX_MOTOR_ACCEL*PERIOD);
        double rampedPercent = currentPercent.getOrDefault(motorGroup, 0.0) + rampedPercentAccel;
        setMotorOutput(motorGroup, rampedPercent);
        currentPercent.put(motorGroup, rampedPercent);
    }


    // DRIVE COMMAND FUNCTIONS

    private void setMotorOutput(MotorGroup group, double pct) {
        if(Math.abs(pct) < MIN_MOTOR_OUTPUT) pct = 0;
        pct = clamp(pct, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT);
        group.getGroup().set(pct);
    }
    public void setMotorVoltage(MotorGroup group, double voltage) {
        setMotorOutput(group, voltage/12.0);
    }
    public void setMotorOutput(double left, double right) {
        setMotorOutput(LEFT, left);
        setMotorOutput(RIGHT, right);
    }
    public void setMotorVoltage(double left, double right) {
        setMotorVoltage(LEFT, left);
        setMotorVoltage(RIGHT, right);
    }
	public void setGoalOutput(double left, double right) {
        goalVelocity.put(LEFT, clamp(left, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT));
        goalVelocity.put(RIGHT, clamp(right, -MAX_MOTOR_OUTPUT, MAX_MOTOR_OUTPUT));
	}

    public void setDriveMode(DriveMode mode) {
        goalVelocity.clear();
        CURRENT_DRIVE_MODE = mode;
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

    public void setDisplacement(Vector3 displacement, double initialAngle) {
        resetEncoders();
        navX.reset();
        odometry = new DifferentialDriveOdometry(
            getRawRotation(),
            new Pose2d(displacement.toTranslation2d(), Rotation2d.fromDegrees(initialAngle))
        );
        this.initialAngle = initialAngle;
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public Vector3 getDisplacement() {
        return position.clone();
    }
    public Vector3 getDisplacement(FieldObject fieldObject) {
        return fieldObject.getPosition().subtract(getDisplacement());
    }
    public Vector3 getVelocity() {
        return position.clone().subtract(lastPosition).multiply(PERIOD);
    }
    public double getRotation() {
        return Math.floorMod((int)(initialAngle + navX.getAngle()), 360);
    }
    private Rotation2d getRawRotation() {
        return Rotation2d.fromDegrees(navX.getAngle());
    }
    public double getWheelDistance(MotorGroup motorGroup) {
        double totalUnits = 0;
        for(WPI_TalonFX motor : motorGroup.getMotors()) {
            System.out.println("distance: "+motor.getSelectedSensorPosition());
            totalUnits += motor.getSelectedSensorPosition();
        }
        double revolutions = totalUnits / 2048.0 / motorGroup.getMotors().length;
        return revolutions * WHEEL_CIRCUMFERENCE;
    }
    public double getWheelVelocity(MotorGroup motorGroup) {
        double totalUnits = 0;
        for(WPI_TalonFX motor : motorGroup.getMotors()) {
            System.out.println("velocity: "+motor.getSelectedSensorVelocity());
            totalUnits += motor.getSelectedSensorVelocity();
        }
        double revolutions = totalUnits / 2048.0 * 10.0 / motorGroup.getMotors().length;
        return revolutions * WHEEL_CIRCUMFERENCE;
    }
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getWheelVelocity(LEFT), getWheelVelocity(RIGHT));
    }
    public void resetEncoders() {
        for(WPI_TalonFX motor : ALL.getMotors()) {
            motor.setSelectedSensorPosition(0);
        }
    }



    // ENUMS

    public enum MotorControlMode {
        FEEDFORWARD,
        PID;

        public Consumer<MotorGroup> getMotorControlFunction(DriveTrainSubsystem instance) {
            switch(this) {
                case PID:
                    return instance::drivePID;
                case FEEDFORWARD:
                    return instance::driveFeedforward;
            }
            return null;
        }
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
        private SpeedControllerGroup group;

        private MotorGroup(WPI_TalonFX[] motorList) {
            this.motorList = motorList;
            this.group = new SpeedControllerGroup(motorList[0], motorList);
        }

        public WPI_TalonFX[] getMotors() {
            return motorList;
        }
        public SpeedControllerGroup getGroup() {
            return group;
        }
    }

    public enum FieldObject {
        POWER_PORT(POWER_PORT_POSITION),
        LOADING_BAY(LOADING_BAY_POSITION);

        private final Vector3 position;
        private FieldObject(Vector3 position) {
            this.position = position;
        }

        public Vector3 getPosition() {
            return position.clone();
        }
    }
}
