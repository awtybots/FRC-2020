/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;
import static frc.robot.Constants.DriveTrain.*;

import java.util.HashMap;

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

	private double goalVelocityLeft = 0;
	private double goalVelocityRight = 0;
	
	@SuppressWarnings("unused") private double outputLeft = 0;
	@SuppressWarnings("unused") private double outputRight = 0;

	private HashMap<MotorGroup, Double> lastVelocityError = new HashMap<>();
	private HashMap<MotorGroup, Double> integralError = new HashMap<>();

	private static double PERIOD;

	public DriveTrainSubsystem() {
		PERIOD = Robot.getTimePeriod();

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
		if(!TUNING_MODE) {
			outputLeft = calculateFF(MotorGroup.LEFT, goalVelocityLeft);
			outputRight = calculateFF(MotorGroup.RIGHT, goalVelocityRight);
			
			//speedLeft.setVoltage(outputLeft);
			//speedRight.setVoltage(outputRight);
		}
	}
	public void set(double speed) {
		speedLeft.set(speed);
		speedRight.set(speed);
		SmartDashboard.putNumber("Voltage", speed * RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("Velocity", getVelocity(MotorGroup.ALL, true));
	}

    @SuppressWarnings("unused")
	private double calculatePID(MotorGroup motorGroup, double goalVelocity) { // this is my best understanding of PID, not sure how accurate this is
		double currentVelocity = getVelocity(motorGroup, false);
		double velocityError = goalVelocity - currentVelocity;
		double accelerationError = (velocityError - lastVelocityError.getOrDefault(motorGroup, 0.0)) / PERIOD;
		lastVelocityError.put(motorGroup, velocityError);
		integralError.put(motorGroup, clamp(integralError.getOrDefault(motorGroup, 0.0) + (velocityError * PERIOD), INTEGRAL_MIN / PID_I, INTEGRAL_MAX / PID_I));
		return (PID_P * velocityError) + (PID_I * integralError.get(motorGroup)) + (PID_D * accelerationError);
	}

    @SuppressWarnings("unused")
	private double calculateFF(MotorGroup motorGroup, double goalVelocity) {
		double constrainedGoalVelocity = clamp(goalVelocity, -MAX_VELOCITY, MAX_VELOCITY);
		double currentVelocity = getVelocity(motorGroup, false);
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
		SmartDashboard.putNumber("Voltage", voltage);
		
		return voltage;
	}

	

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



	public double getVelocity(MotorGroup motorGroup, boolean abs) { // utility function to get average inches per second from certain groups of motors
		double totalUnitsPer100ms = 0;
		for(WPI_TalonSRX motor : motorGroup.getMotors()) {
			double motorVelocity = motor.getSelectedSensorVelocity();
			totalUnitsPer100ms += abs ? Math.abs(motorVelocity) : motorVelocity;
		}
		// encoders give motor velocity in units per 100ms, so multiply by 10 for units per second and divide by units per rev for revs per second
		// multiply revolutions per second by seconds elapsed and wheel circumference for distance traveled
		double averageUnitsPer100ms = totalUnitsPer100ms / motorGroup.getMotors().length;
		double revolutionsPerSecond = averageUnitsPer100ms / 409.6 * GEAR_RATIO;
		return revolutionsPerSecond * WHEEL_CIRCUMFERENCE;
	}
	public double getDistance(boolean abs) {
		double totalUnits = 0;
		for(WPI_TalonSRX motor : MotorGroup.ALL.getMotors()) {
			double pos = motor.getSelectedSensorPosition();
			totalUnits += abs ? Math.abs(pos) : pos;
		}
		double revolutions = totalUnits / 4096 / MotorGroup.ALL.getMotors().length;
		return revolutions * WHEEL_CIRCUMFERENCE;
	}
	public void resetDistance() {
		for(WPI_TalonSRX motor : MotorGroup.ALL.getMotors()) {
			motor.setSelectedSensorPosition(0);
		}
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
}