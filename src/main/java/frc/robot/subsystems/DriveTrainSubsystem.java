/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class DriveTrainSubsystem extends SubsystemBase {//PIDSubsystem { 

	// this is the subsystem that interacts with the motors

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

	public DriveTrainSubsystem() {
		forEachMotor((motor) -> motor.setNeutralMode(Constants.BRAKE_MODE)); // sets the brake mode for all motors (called NeutralMode)
	}

	@Override
	public void periodic() {

	}

	public void arcadeDrive(double xSpeed, double zRotation) {
		differentialDrive.arcadeDrive(xSpeed, zRotation); // DifferentialDrive has a built-in arcadeDrive function
	}



	public void stop() {
		forEachMotor((motor) -> motor.stopMotor()); // stops all motors
	}

	private void forEachMotor(Consumer<WPI_TalonSRX> consumer) { // utility function to do something for every motor
		for(WPI_TalonSRX motor : motors) {
			consumer.accept(motor);
		}
	}

	/*
	@Override
	protected void useOutput(double output, double setpoint) {
		// TODO Auto-generated method stub

	}

	@Override
	protected double getMeasurement() {
		// TODO Auto-generated method stub
		return 0;
	}
	*/
}