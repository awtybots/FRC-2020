/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private XboxController xboxController;
	private DriveTrainSubsystem driveTrainSubsystem;
	private Teleop teleopCommand;
	private Auton autonCommand;

	@Override
	public void robotInit() {
		// Subsystems
		xboxController = new XboxController(Ports.XBOX_CONTROLLER);
		driveTrainSubsystem = new DriveTrainSubsystem();

		// Commands
		teleopCommand = new Teleop(xboxController, driveTrainSubsystem);
		autoCommand = new Auton(driveTrainSubsystem);

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
		autonCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		autonCommand.cancel();

		teleopCommand.schedule();

		// Uncomment this if robot does not drive
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
