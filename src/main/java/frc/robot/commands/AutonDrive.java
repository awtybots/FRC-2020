/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDrive extends CommandBase {

	private final DriveTrainSubsystem driveTrainSubsystem;

	public AutonDrive(DriveTrainSubsystem driveTrainSubsystem) {
		this.driveTrainSubsystem = driveTrainSubsystem;
		addRequirements(driveTrainSubsystem);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
