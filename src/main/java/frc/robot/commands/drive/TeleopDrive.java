/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.Controller.*;
import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Robot.*;

public class TeleopDrive extends CommandBase {

	// this command runs the entire teleop period

	public TeleopDrive() {
		addRequirements(driveTrainSubsystem);
	}

	private double smooth(double x) {
		if(Math.abs(x) < DEADZONE) return 0;
		return Math.pow(x, 2) * Math.signum(x);
	}
	
	@Override
	public void execute() {
		double speed = smooth(-xboxController1.getY(SPEED_HAND));
		double rotation = smooth(xboxController1.getX(ROTATION_HAND));
		double left = speed + rotation;
		double right = speed - rotation;
		switch(DRIVE_MODE) {
			case DIRECT:
				driveTrainSubsystem.setMotorOutput(left * MAX_TELEOP_MOTOR_OUTPUT, right * MAX_TELEOP_MOTOR_OUTPUT);
				break;
			case SMOOTH:
				driveTrainSubsystem.setGoalVelocity(left * MAX_VELOCITY, right * MAX_VELOCITY);
				break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		driveTrainSubsystem.smoothStop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}