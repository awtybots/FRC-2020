/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;
import frc.robot.Robot.GamePeriod;
import frc.robot.util.Vector3;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Constants.NavX.*;

import java.util.HashMap;
import java.util.function.BiFunction;

public class DriveTrainSubsystem extends SubsystemBase {

    // this is the subsystem that interacts with the drivetrain motors

    private final static WPI_TalonSRX motorL1 = new WPI_TalonSRX(MotorIDs.DRIVE_L1);
    private final static WPI_TalonSRX motorL2 = new WPI_TalonSRX(MotorIDs.DRIVE_L2);
    private final static WPI_TalonSRX motorL3 = new WPI_TalonSRX(MotorIDs.DRIVE_L3);

    private final static WPI_TalonSRX motorR1 = new WPI_TalonSRX(MotorIDs.DRIVE_R1);
    private final static WPI_TalonSRX motorR2 = new WPI_TalonSRX(MotorIDs.DRIVE_R2);
    private final static WPI_TalonSRX motorR3 = new WPI_TalonSRX(MotorIDs.DRIVE_R3);

    private final static SpeedControllerGroup speedLeft = new SpeedControllerGroup(motorL1, motorL2, motorL3);
    private final static SpeedControllerGroup speedRight = new SpeedControllerGroup(motorR1, motorR2, motorR3);

	private final BiFunction<MotorGroup, Double, Double> motorControlFunction;

	private double goalVelocityLeft = 0;
	private double goalVelocityRight = 0;
	
	private double outputLeft = 0;
	private double outputRight = 0;

    private HashMap<MotorGroup, Double> lastVelocityError = new HashMap<>();
    private HashMap<MotorGroup, Double> integralError = new HashMap<>();

	private final AHRS board = new AHRS(SPI.Port.kMXP);
	
	private DifferentialDriveOdometry odometry;
	private Vector3 lastPosition;
	private Vector3 position;

	private double initialAngle;

	private static double PERIOD;


	// DRIVING

	public DriveTrainSubsystem() {
		PERIOD = Robot.getLoopTime();
		motorControlFunction = MOTOR_CONTROL_MODE == MotorControlMode.FEEDFORWARD ? this::calculateFF : this::calculatePID;

        for (WPI_TalonSRX motor : MotorGroup.ALL.getMotors()) {
            motor.set(0); // start all motors at 0% speed to stop the blinking

            motor.configFactoryDefault(); // reset settings
            motor.setNeutralMode(BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
            motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative); // sets which encoder the motor is using
        }

        for (WPI_TalonSRX motor : MotorGroup.RIGHT.getMotors()) {
            motor.setSensorPhase(true);
        }

        speedRight.setInverted(true);
    }

	@Override
	public void periodic() {
		if(Robot.getGamePeriod() == GamePeriod.AUTON || DRIVE_MODE == DriveTrainSubsystem.DriveMode.SMOOTH) {
			outputLeft = motorControlFunction.apply(MotorGroup.LEFT, goalVelocityLeft);
			outputRight = motorControlFunction.apply(MotorGroup.RIGHT, goalVelocityRight);

			SmartDashboard.putNumber("Voltage left", outputLeft);
			SmartDashboard.putNumber("Voltage right", outputRight);

			speedLeft.setVoltage(outputLeft);
			speedRight.setVoltage(outputRight);
		}
		
		odometry.update(getRawRotation(), getWheelDistance(MotorGroup.LEFT, false), getWheelDistance(MotorGroup.RIGHT, false));
		lastPosition = position.clone();
		position = new Vector3(odometry.getPoseMeters()).print("Position");
		getVelocity().print("Velocity");
	}

	private double calculatePID(MotorGroup motorGroup, double goalVelocity) {
		double currentVelocity = getWheelVelocity(motorGroup, false);
		double velocityError = goalVelocity - currentVelocity;
		double accelerationError = (velocityError - lastVelocityError.getOrDefault(motorGroup, 0.0)) / PERIOD;
		lastVelocityError.put(motorGroup, velocityError);
		integralError.put(motorGroup, clamp(integralError.getOrDefault(motorGroup, 0.0) + (velocityError * PERIOD), INTEGRAL_MIN / PID_I, INTEGRAL_MAX / PID_I));
		return (PID_P * velocityError) + (PID_I * integralError.get(motorGroup)) + (PID_D * accelerationError);
	}

	private double calculateFF(MotorGroup motorGroup, double goalVelocity) {
		double constrainedGoalVelocity = clamp(goalVelocity, -MAX_VELOCITY, MAX_VELOCITY);
		double currentVelocity = getWheelVelocity(motorGroup, false);
		double goalAcceleration = constrainedGoalVelocity - currentVelocity;
		double constrainedGoalAcceleration = clamp(goalAcceleration, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD);
		constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

		double S = FF_S * Math.signum(constrainedGoalVelocity);
		double V = FF_V * constrainedGoalVelocity;
		double A = FF_A * constrainedGoalAcceleration;
		double voltage = S + V + A;

		SmartDashboard.putNumber("Current Velocity", currentVelocity);
		SmartDashboard.putNumber("Goal Velocity", constrainedGoalVelocity);
		SmartDashboard.putNumber("Goal Acceleration", constrainedGoalAcceleration);
		SmartDashboard.putNumber("FF_S", S);
		SmartDashboard.putNumber("FF_V", V);
		SmartDashboard.putNumber("FF_A", A);
		
		return voltage;
	}


	// DRIVE COMMAND FUNCTIONS

	public void setMotorOutput(double left, double right) {
		speedLeft.set(left);
		speedRight.set(right);
	}
	public void setGoalVelocity(double left, double right) {
		goalVelocityLeft = left;
		goalVelocityRight = right;
	}
	public void smoothStop() {
		setGoalVelocity(0, 0);
	}
	public void hardStop() {
		setGoalVelocity(0, 0);
		setMotorOutput(0, 0);
	}


	// SENSORS

    public void setDisplacement(Vector3 displacement, double initialAngle) {
		resetEncoders();
        board.reset();
		odometry = new DifferentialDriveOdometry(
			getRawRotation(),
			new Pose2d(displacement.toTranslation2d(), Rotation2d.fromDegrees(initialAngle))
		);
		this.initialAngle = allianceCondition(initialAngle);
	}
	public Vector3 getDisplacement() {
		return position;
	}
	public Vector3 getDisplacement(FieldObject fieldObject) {
		return fieldObject.getPosition().subtract(getDisplacement());
	}
    public Vector3 getVelocity() {
        return position.subtract(lastPosition).multiply(PERIOD);
	}
	public double getRotation() {
		return Math.floorMod((int)(initialAngle - board.getAngle()), 360);
	}


	private Rotation2d getRawRotation() {
		return Rotation2d.fromDegrees(-board.getAngle());
	}
	public double getWheelDistance(MotorGroup motorGroup, boolean abs) {
		double totalUnits = 0;
		for(WPI_TalonSRX motor : motorGroup.getMotors()) {
			double pos = motor.getSelectedSensorPosition();
			totalUnits += abs ? Math.abs(pos) : pos;
		}
		double revolutions = totalUnits / 4096 / motorGroup.getMotors().length;
		return revolutions * WHEEL_CIRCUMFERENCE;
	}
	public double getWheelVelocity(MotorGroup motorGroup, boolean abs) {
		double totalUnits = 0;
		for(WPI_TalonSRX motor : motorGroup.getMotors()) {
			double vel = motor.getSelectedSensorVelocity();
			totalUnits += abs ? Math.abs(vel) : vel;
		}
		double revolutions = totalUnits / 409.6 / motorGroup.getMotors().length;
		return revolutions * WHEEL_CIRCUMFERENCE;
	}
	public void resetEncoders() {
		for(WPI_TalonSRX motor : MotorGroup.ALL.getMotors()) {
			motor.setSelectedSensorPosition(0);
		}
	}

	
	// UTILITIES

    private static Vector3 allianceCondition(Vector3 blue) {
        return Robot.getAlliance() == Alliance.Blue ? blue : blue.rotateZ(180).add(new Vector3(FIELD_WIDTH, 0, 0));
    }
    private static double allianceCondition(double blue) {
        return Robot.getAlliance() == Alliance.Blue ? blue : blue + 180;
    }


	// ENUMS

	public enum MotorControlMode {
		FEEDFORWARD,
		PID;
	}

	public enum DriveMode {
		DIRECT,
		SMOOTH;
	}

	public enum MotorGroup {
		LEFT(new WPI_TalonSRX[]{ motorL1, motorL2, motorL3 }),
		RIGHT(new WPI_TalonSRX[]{ motorR1, motorR2, motorR3 }),
		ALL(new WPI_TalonSRX[]{ motorL1, motorL2, motorL3, motorR1, motorR2, motorR3 });

		private WPI_TalonSRX[] motorList;

		private MotorGroup(WPI_TalonSRX[] motorList) {
			this.motorList = motorList;
		}

		public WPI_TalonSRX[] getMotors() {
			return motorList;
		}
	}

    public enum FieldObject {
		POWER_PORT(POWER_PORT_POSITION),
		LOADING_BAY(LOADING_BAY_POSITION);

        private final Vector3 position;
        private FieldObject(Vector3 position) {
            this.position = allianceCondition(position);
        }

        public Vector3 getPosition() {
            return position;
        }
    }
}