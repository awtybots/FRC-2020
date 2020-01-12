/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.Values;

public class DriveTrainSubsystem extends SubsystemBase { 

	// this is the subsystem that interacts with the drivetrain motors

	private final WPI_TalonSRX motorL1 = new WPI_TalonSRX(MotorIDs.MOTOR_L1);
	private final WPI_TalonSRX motorL2 = new WPI_TalonSRX(MotorIDs.MOTOR_L2);
	private final WPI_TalonSRX motorL3 = new WPI_TalonSRX(MotorIDs.MOTOR_L3);

	private final WPI_TalonSRX motorR1 = new WPI_TalonSRX(MotorIDs.MOTOR_R1);
	private final WPI_TalonSRX motorR2 = new WPI_TalonSRX(MotorIDs.MOTOR_R2);
	private final WPI_TalonSRX motorR3 = new WPI_TalonSRX(MotorIDs.MOTOR_R3);

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



	private double driveInchesGoal;
	private double driveInchesProgress = 0;
	private double turnDegreesGoal;
	private double turnDegreesProgress = 0;
	private Timer timer;
	private double lastTime;



	public DriveTrainSubsystem() {
		forEachMotor((motor) -> {
			motor.set(ControlMode.PercentOutput, 0); // start all motors at 0% speed to stop the blinking

			motor.configFactoryDefault();
			motor.setNeutralMode(Values.BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
			motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
			motor.setSensorPhase(true);
		});
	}

	@Override
	public void periodic() {

	}

	public void arcadeDrive(double speed, double rotation) {
		differentialDrive.arcadeDrive(speed, rotation); // DifferentialDrive has a built-in arcadeDrive function
	}

	public void driveInchesInitialize(double inches) {
		driveInchesGoal = inches;
		driveInchesProgress = 0;
		timer = new Timer();
		timer.start();
		lastTime = timer.get();

		arcadeDrive(Values.AUTON_DRIVE_SPEED, 0);
	}

	public void driveInchesExecute() {
		final double[] motorVelocityAverage = new double[]{0}; // using an array to bypass the arrow function "must be final" error
		forEachMotor((motor) -> motorVelocityAverage[0] += motor.getSelectedSensorVelocity());
		motorVelocityAverage[0] /= motors.length;

		double currentTime = timer.get();
		double elapsedTime = currentTime - lastTime;
		lastTime = currentTime;

		driveInchesProgress += elapsedTime * motorVelocityAverage[0] * 10.0 / Values.ENCODER_UNITS;
	}

	public boolean driveInchesIsFinished() {
		return driveInchesProgress > driveInchesGoal;
	}



	public void stop() {
		forEachMotor((motor) -> motor.stopMotor()); // stops all motors
	}

	private void forEachMotor(Consumer<WPI_TalonSRX> consumer) { // utility function to do something for every motor
		for(WPI_TalonSRX motor : motors) {
			consumer.accept(motor);
		}
	}
}