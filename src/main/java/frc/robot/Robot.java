/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.Auton.AutonType;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

	private XboxController xboxController;
	private DriveTrainSubsystem driveTrainSubsystem;
	private Teleop teleopCommand;
	private Auton autonCommand;

	private SendableChooser<AutonType> autonChooser;

	@Override
	public void robotInit() {
		// Auton chooser
		autonChooser = new SendableChooser<>();
		AutonType[] autonTypes = AutonType.values();
		for(AutonType autonType : autonTypes) {
			autonChooser.addOption(autonType.toString(), autonType);
		}

		// Subsystems
		xboxController = new XboxController(Ports.XBOX_CONTROLLER); // xbox controller has no family so subsystems adopted it
		driveTrainSubsystem = new DriveTrainSubsystem();

		// Commands
		teleopCommand = new Teleop(xboxController, driveTrainSubsystem); // overlaying teleop command for the teleop period

		// Button Mappings


	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void autonomousInit() {
		autonCommand = new Auton(driveTrainSubsystem, autonChooser.getSelected()); // get chosen AutonType
		autonCommand.schedule(); // start auton
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		autonCommand.cancel(); // finish auton

		teleopCommand.schedule(); // start teleop

		// Alex's code
		// Command teleopCommand = driveTrainSubsystem.getDefaultCommand();
		// teleopCommand.schedule();
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {

	}
}
