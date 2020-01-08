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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;

public class DriveTrainSubsystem extends SubsystemBase {

	private final WPI_TalonSRX motorL1 = new WPI_TalonSRX(MotorIDs.MOTOR_L1);
	private final WPI_TalonSRX motorL2 = new WPI_TalonSRX(MotorIDs.MOTOR_L2);
	private final WPI_TalonSRX motorL3 = new WPI_TalonSRX(MotorIDs.MOTOR_L3);

	private final WPI_TalonSRX motorR1 = new WPI_TalonSRX(MotorIDs.MOTOR_R1);
	private final WPI_TalonSRX motorR2 = new WPI_TalonSRX(MotorIDs.MOTOR_R2);
	private final WPI_TalonSRX motorR3 = new WPI_TalonSRX(MotorIDs.MOTOR_R3);

	private final SpeedControllerGroup speedLeft = new SpeedControllerGroup(motorL1, motorL2, motorL3);
	private final SpeedControllerGroup speedRight = new SpeedControllerGroup(motorR1, motorR2, motorR3);

	private final DifferentialDrive differentialDrive = new DifferentialDrive(speedLeft, speedRight);

	private final WPI_TalonSRX[] motors = new WPI_TalonSRX[]{
		motorL1,
		motorL2,
		motorL3,
		motorR1,
		motorR2,
		motorR3,
	};

	public DriveTrainSubsystem() {
		forEachMotor((motor) -> motor.setNeutralMode(Constants.BRAKE_MODE));
	}

	@Override
	public void periodic() {

	}

	public void arcadeDrive(double xSpeed, double zRotation) {
		differentialDrive.arcadeDrive(xSpeed, zRotation);
	}

	public void stop() {
		forEachMotor((motor) -> motor.stopMotor());
	}

	private void forEachMotor(Consumer<WPI_TalonSRX> consumer) {
		for(WPI_TalonSRX motor : motors) {
			consumer.accept(motor);
		}
	}
}