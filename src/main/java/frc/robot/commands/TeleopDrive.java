/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Controller;

public class TeleopDrive extends CommandBase {

	private final XboxController xboxController;
	private final DriveTrainSubsystem driveTrainSubsystem;

	public TeleopDrive(XboxController xboxController, DriveTrainSubsystem driveTrainSubsystem) {
		this.xboxController = xboxController;
		this.driveTrainSubsystem = driveTrainSubsystem;
		addRequirements(driveTrainSubsystem);
	}

	@Override
	public void initialize() {
		
	}

	@Override
	public void execute() {
		double xSpeed = xboxController.getY(Controller.SPEED_HAND);
		double zRotation = xboxController.getX(Controller.ROTATION_HAND);
		driveTrainSubsystem.arcadeDrive(xSpeed, zRotation);
	}

	@Override
	public void end(boolean interrupted) {
		driveTrainSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}