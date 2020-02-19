/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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

    private static WPI_TalonFX motorL1;
    private static WPI_TalonFX motorL2;

    private static WPI_TalonFX motorR1;
    private static WPI_TalonFX motorR2;

    private HashMap<MotorGroup, Double> goalVelocity = new HashMap<>();
    private HashMap<MotorGroup, Double> lastVelocityError = new HashMap<>();
    private HashMap<MotorGroup, Double> integralError = new HashMap<>();

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private DifferentialDriveOdometry odometry;
    private Vector3 lastPosition = new Vector3();
    private Vector3 position = new Vector3();

    private double initialAngle;

    private static double PERIOD;


    // DRIVING

    public DriveTrainSubsystem() {
        PERIOD = Robot.getLoopTime();

        motorL1 = new WPI_TalonFX(MotorIDs.DRIVE_L1);
        motorL2 = new WPI_TalonFX(MotorIDs.DRIVE_L2);

        motorR1 = new WPI_TalonFX(MotorIDs.DRIVE_R1);
        motorR2 = new WPI_TalonFX(MotorIDs.DRIVE_R2);

        for (WPI_TalonFX motor : ALL.getMotors()) {
            motor.set(0); // start all motors at 0% speed to stop the blinking

            motor.configFactoryDefault(); // reset settings
            motor.setNeutralMode(BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // sets which encoder the motor is using
        }

        for (WPI_TalonFX motor : RIGHT.getMotors()) {
            motor.setSensorPhase(true);
        }

        RIGHT.getGroup().setInverted(true);
    }

    @Override
    public void periodic() {
        if(DRIVE_MODE != DriveTrainSubsystem.DriveMode.DIRECT) {
            MOTOR_CONTROL_MODE.getMotorControlFunction(this).accept(LEFT);
            MOTOR_CONTROL_MODE.getMotorControlFunction(this).accept(RIGHT);
        }

        if(odometry != null) {
            odometry.update(getRawRotation(), getWheelDistance(LEFT), getWheelDistance(RIGHT));
            lastPosition = position.clone();
            position = new Vector3(odometry.getPoseMeters()).print("Position");
            getVelocity().print("Velocity");
            SmartDashboard.putNumber("Rotation", getRotation());
        }
    }

    private void drivePID(MotorGroup motorGroup) {
        double currentVelocity = getWheelVelocity(motorGroup);
        double goalAcceleration = goalVelocity.getOrDefault(motorGroup, 0.0) - currentVelocity;
        double velocityError = DRIVE_MODE == DriveMode.TRAPEZOIDAL_VELOCITY
            ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
            : goalAcceleration;
        double accelerationError = (velocityError - lastVelocityError.getOrDefault(motorGroup, 0.0)) / PERIOD;
        lastVelocityError.put(motorGroup, velocityError);
        integralError.put(motorGroup, clamp(integralError.getOrDefault(motorGroup, 0.0) + (velocityError * PERIOD), -INTEGRAL_MAX / PID_I, INTEGRAL_MAX / PID_I));

        double P = PID_P * velocityError;
        double I = PID_I * integralError.get(motorGroup);
        double D = PID_D * accelerationError;

        double output = P + I + D;
        motorGroup.getGroup().set(output);
    }

    private void driveFeedforward(MotorGroup motorGroup) {
        double currentVelocity = getWheelVelocity(motorGroup);
        double goalAcceleration = goalVelocity.getOrDefault(motorGroup, 0.0) - currentVelocity;
        double constrainedGoalAcceleration = DRIVE_MODE == DriveMode.TRAPEZOIDAL_VELOCITY
            ? clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD)
            : goalAcceleration;
        double constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

        double S = FF_S * Math.signum(constrainedGoalVelocity);
        double V = FF_V * constrainedGoalVelocity;
        double A = FF_A * constrainedGoalAcceleration;

        double voltage = S + V + A;
        motorGroup.getGroup().setVoltage(voltage);
    }


    // DRIVE COMMAND FUNCTIONS

    public void setMotorOutput(double left, double right) {
        if(Math.abs(left) < MIN_MOTOR_OUTPUT) left = 0;
        if(Math.abs(right) < MIN_MOTOR_OUTPUT) right = 0;
        LEFT.getGroup().set(left);
        RIGHT.getGroup().set(right);
    }
    public void setMotorVoltage(double left, double right) {
        if(Math.abs(left) < MIN_MOTOR_OUTPUT * 12.0) left = 0;
        if(Math.abs(right) < MIN_MOTOR_OUTPUT * 12.0) right = 0;
        LEFT.getGroup().setVoltage(left);
        RIGHT.getGroup().setVoltage(right);
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
            totalUnits += motor.getSelectedSensorPosition();
        }
        double revolutions = totalUnits / 2048.0 / motorGroup.getMotors().length;
        return revolutions * WHEEL_CIRCUMFERENCE;
    }
    public double getWheelVelocity(MotorGroup motorGroup) {
        double totalUnits = 0;
        for(WPI_TalonFX motor : motorGroup.getMotors()) {
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
        DIRECT,
        VELOCITY,
        TRAPEZOIDAL_VELOCITY;
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
