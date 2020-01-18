/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Controller.*;
import static frc.robot.Constants.DriveTrain.*;

public class TeleopDrive extends CommandBase {

	// this command runs the entire teleop period

	private final XboxController xboxController;
	private final DriveTrainSubsystem driveTrainSubsystem;

	public TeleopDrive(XboxController xboxController, DriveTrainSubsystem driveTrainSubsystem) {
		addRequirements(driveTrainSubsystem);
		this.xboxController = xboxController;
		this.driveTrainSubsystem = driveTrainSubsystem;
		if(TUNING_MODE) {
			SmartDashboard.setDefaultNumber("Test Speed", 0);
		}
	}

	private double smooth(double x) {
		if(Math.abs(x) < DEADZONE) return 0;
		return Math.pow(x, 2) * Math.signum(x);
	}
	
	@Override
	public void execute() {
		if(TUNING_MODE) {
			double speed = SmartDashboard.getNumber("Test Speed", 0);
			driveTrainSubsystem.set(speed);
		} else {
			double speed = smooth(-xboxController.getY(SPEED_HAND));
			double rotation = smooth(xboxController.getX(ROTATION_HAND));
			double left = speed + rotation;
			double right = speed - rotation;
			driveTrainSubsystem.setMotorOutput(left * MAX_TELEOP_MOTOR_OUTPUT, right * MAX_TELEOP_MOTOR_OUTPUT);
			//driveTrainSubsystem.setGoalVelocity((speed + rotation) * MAX_VELOCITY, (speed - rotation) * MAX_VELOCITY);
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