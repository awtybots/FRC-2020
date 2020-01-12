/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.DriveTrain;

public class DriveTrainSubsystem extends SubsystemBase { 

	// this is the subsystem that interacts with the drivetrain motors

	private final WPI_TalonSRX motorL1 = new WPI_TalonSRX(MotorIDs.DRIVE_L1);
	private final WPI_TalonSRX motorL2 = new WPI_TalonSRX(MotorIDs.DRIVE_L2);
	private final WPI_TalonSRX motorL3 = new WPI_TalonSRX(MotorIDs.DRIVE_L3);

	private final WPI_TalonSRX motorR1 = new WPI_TalonSRX(MotorIDs.DRIVE_R1);
	private final WPI_TalonSRX motorR2 = new WPI_TalonSRX(MotorIDs.DRIVE_R2);
	private final WPI_TalonSRX motorR3 = new WPI_TalonSRX(MotorIDs.DRIVE_R3);

	private final SpeedControllerGroup speedLeft = new SpeedControllerGroup(motorL1, motorL2, motorL3); // all left motors in one group
	private final SpeedControllerGroup speedRight = new SpeedControllerGroup(motorR1, motorR2, motorR3); // all right motors in another

	private final DifferentialDrive differentialDrive = new DifferentialDrive(speedLeft, speedRight); // helpful class that has lots of driving modes built-in

	private final WPI_TalonSRX[] motors = new WPI_TalonSRX[]{
		motorL1,
		motorL2,
		motorL3,
		motorR1,
		motorR2,
		motorR3,
	};

	private Direction driveInchesDirection;
	private double driveInchesProgress;
	private Direction rotateDegreesDirection;
	private double rotateDegreesProgress;

	private Timer timer = new Timer();
	private double lastTime;

	public DriveTrainSubsystem() {
		forEachMotor((motor) -> {
			motor.set(ControlMode.PercentOutput, 0); // start all motors at 0% speed to stop the blinking

			motor.configFactoryDefault();
			motor.setNeutralMode(DriveTrain.BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
			motor.configSelectedFeedbackSensor(DriveTrain.MOTOR_FEEDBACK_DEVICE); // sets which encoder the motor is using
		});
	}



	public void arcadeDrive(double speed, double rotation) {
		differentialDrive.arcadeDrive(-speed, rotation); // DifferentialDrive has a built-in arcadeDrive function
	}
	public void stop() {
		arcadeDrive(0, 0);
	}



	public void driveInchesInitialize(double inches) {
		driveInchesDirection = (inches > 0) ? Direction.FORWARD : Direction.BACKWARD;
		driveInchesProgress = 0;

		timer.reset();
		timer.start();
		lastTime = 0;
	}
	public double driveInchesGetMeasurement() {
		double currentTime = timer.get();
		double elapsedTime = currentTime - lastTime;
		lastTime = currentTime;
		
		double inchesTraveled = getAverageInchesPerSecond() * elapsedTime;
		driveInchesProgress += inchesTraveled;

		return driveInchesProgress;
	}
	public void driveInchesUseOutput(double output, TrapezoidProfile.State state) {
		arcadeDrive(DriveTrain.AUTON_DRIVE_SPEED * output * driveInchesDirection.getMultiplier(), 0);
	}



	public void rotateDegreesInitialize(double degrees) {
		rotateDegreesDirection = (degrees > 0) ? Direction.CLOCKWISE : Direction.COUNTER_CLOCKWISE;
		rotateDegreesProgress = 0;

		timer.reset();
		timer.start();
		lastTime = 0;
	}
	public double rotateDegreesGetMeasurement() {
		double currentTime = timer.get();
		double elapsedTime = currentTime - lastTime;
		lastTime = currentTime;

		double inchesRotated = getAverageInchesPerSecond() * elapsedTime;
		double degreesRotated = inchesRotated / DriveTrain.ROBOT_CIRMCUMFERENCE * 360.0;
		rotateDegreesProgress += degreesRotated;

		return rotateDegreesProgress;
	}
	public void rotateDegreesUseOutput(double output, TrapezoidProfile.State state) {
		arcadeDrive(0, DriveTrain.AUTON_ROTATE_SPEED * output * rotateDegreesDirection.getMultiplier());
	}



	public enum Direction {
		CLOCKWISE(1.0),
		COUNTER_CLOCKWISE(-1.0),
		FORWARD(1.0),
		BACKWARD(-1.0);

		private double dir;

		private Direction(double dir) {
			this.dir = dir;
		}

		public double getMultiplier() {
			return dir;
		}
	}



	private double getAverageInchesPerSecond() { // utility function to get average inches per second from all motors
		double totalUnitsPer100ms = 0;
		for(WPI_TalonSRX motor : motors) {
			totalUnitsPer100ms += Math.abs(motor.getSelectedSensorVelocity());
		}
		// encoders give motor velocity in units per 100ms, so multiply by 10 for units per second and divide by units for revs per second
		// multiply revolutions per second by seconds elapsed and wheel circumference for distance traveled
		double averageUnitsPer100ms = totalUnitsPer100ms / motors.length;
		double revolutionsPerSecond = averageUnitsPer100ms * 10.0 / DriveTrain.ENCODER_UNITS;
		return revolutionsPerSecond * DriveTrain.WHEEL_CIRCUMFERENCE;
	}



	private void forEachMotor(Consumer<WPI_TalonSRX> consumer) { // utility function to do something for every motor
		for(WPI_TalonSRX motor : motors) {
			consumer.accept(motor);
		}
	}
}