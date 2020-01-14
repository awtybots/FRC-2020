/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;
import frc.robot.Constants.MotorIDs;
import static frc.robot.Constants.DriveTrain.*;

public class DriveTrainSubsystem extends SubsystemBase {

	// this is the subsystem that interacts with the drivetrain motors

	private final static WPI_TalonSRX motorL1 = new WPI_TalonSRX(MotorIDs.DRIVE_L1);
	private final static WPI_TalonSRX motorL2 = new WPI_TalonSRX(MotorIDs.DRIVE_L2);
	private final static WPI_TalonSRX motorL3 = new WPI_TalonSRX(MotorIDs.DRIVE_L3);

	private final static WPI_TalonSRX motorR1 = new WPI_TalonSRX(MotorIDs.DRIVE_R1);
	private final static WPI_TalonSRX motorR2 = new WPI_TalonSRX(MotorIDs.DRIVE_R2);
	private final static WPI_TalonSRX motorR3 = new WPI_TalonSRX(MotorIDs.DRIVE_R3);

	private final static WPI_TalonSRX[] motors = new WPI_TalonSRX[] { motorL1, motorL2, motorL3, motorR1, motorR2, motorR3 };
	private final static WPI_TalonSRX[] leftMotors = new WPI_TalonSRX[] { motorL1, motorL2, motorL3 };
	private final static WPI_TalonSRX[] rightMotors = new WPI_TalonSRX[] { motorR1, motorR2, motorR3 };

	private final static SpeedControllerGroup speedLeft = new SpeedControllerGroup(motorL1, motorL2, motorL3);
	private final static SpeedControllerGroup speedRight = new SpeedControllerGroup(motorR1, motorR2, motorR3);

	private double goalVelocityLeft = 0;
	private double goalVelocityRight = 0;
	private double outputLeft = 0;
	private double outputRight = 0;

	public DriveTrainSubsystem() {
		for (WPI_TalonSRX motor : motors) {
			motor.set(0); // start all motors at 0% speed to stop the blinking

			motor.configFactoryDefault();
			motor.setNeutralMode(BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
			motor.configSelectedFeedbackSensor(MOTOR_FEEDBACK_DEVICE); // sets which encoder the motor is using
		}
	}

	@Override
	public void periodic() {
		outputLeft += calculateFF(MotorGroup.LEFT, goalVelocityLeft);
		outputRight += calculateFF(MotorGroup.RIGHT, goalVelocityRight);

		set(outputLeft, outputRight);
	}

	private void set(double left, double right) {
		speedLeft.setVoltage(left);
		speedRight.setVoltage(right);
	}

	public void setGoalVelocity(double left, double right) {
		goalVelocityLeft = left;
		goalVelocityRight = right;
	}

	public void stop() {
		setGoalVelocity(0, 0);
	}

	public double calculateFF(MotorGroup motorGroup, double goalVelocity) { // this is my best understanding of PID, not sure how accurate this is
		double constrainedGoalVelocity = clamp(goalVelocity, -MAX_VELOCITY, MAX_VELOCITY);
		double currentVelocity = getAverageInchesPerSecond(motorGroup, false);
		double goalAcceleration = constrainedGoalVelocity - currentVelocity;
		double constrainedGoalAcceleration = clamp(goalAcceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
		return (FF_S * Math.signum(currentVelocity)) + (FF_V * currentVelocity) + (FF_A * constrainedGoalAcceleration);
	}

	public double getAverageInchesPerSecond(MotorGroup motorGroup, boolean abs) { // utility function to get average inches per second from certain groups of motors
		double totalUnitsPer100ms = 0;
		for(WPI_TalonSRX motor : motorGroup.getMotors()) {
			double motorVelocity = motor.getSelectedSensorVelocity();
			totalUnitsPer100ms += abs ? Math.abs(motorVelocity) : motorVelocity;
		}
		// encoders give motor velocity in units per 100ms, so multiply by 10 for units per second and divide by units per rev for revs per second
		// multiply revolutions per second by seconds elapsed and wheel circumference for distance traveled
		double averageUnitsPer100ms = totalUnitsPer100ms / motors.length;
		double revolutionsPerSecond = averageUnitsPer100ms * 10.0 / ENCODER_UNITS;
		return revolutionsPerSecond * WHEEL_CIRCUMFERENCE;
	}

	public enum MotorGroup {
		LEFT(leftMotors),
		RIGHT(rightMotors),
		ALL(motors);

		private WPI_TalonSRX[] motorList;

		private MotorGroup(WPI_TalonSRX[] motorList) {
			this.motorList = motorList;
		}

		public WPI_TalonSRX[] getMotors() {
			return motorList;
		}
	}
}