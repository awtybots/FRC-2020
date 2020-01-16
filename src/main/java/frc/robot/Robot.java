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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.controlpanel.AutoSpinControlPanel;
import frc.robot.commands.controlpanel.ToggleControlPanelSpinner;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.main.*;
import frc.robot.commands.main.Auton.AutonType;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

	private XboxController xboxController;

	private DriveTrainSubsystem driveTrainSubsystem;
	private IntakeSubsystem intakeSubsystem;
	private ShooterSubsystem shooterSubsystem;
	private ControlPanelSubsystem controlPanelSubsystem;
	private LimelightSubsystem limelightSubsystem;
	private NavXSubsystem navXSubsystem;

	private Teleop teleopCommand;
	private Auton autonCommand;

	private SendableChooser<AutonType> autonChooser;

	private static double period;

	@Override
	public void robotInit() {
		period = getPeriod();

		// Auton chooser
		autonChooser = new SendableChooser<>();
		AutonType[] autonTypes = AutonType.values();
		for(AutonType autonType : autonTypes) {
			if(autonType == AutonType.DO_NOTHING) {
				autonChooser.setDefaultOption(autonType.toString(), autonType);
			} else {
				autonChooser.addOption(autonType.toString(), autonType);
			}
		}
		SmartDashboard.putData(autonChooser);

		// Subsystems
		xboxController = new XboxController(Controller.PORT);
		driveTrainSubsystem = new DriveTrainSubsystem();
		intakeSubsystem = new IntakeSubsystem();
		shooterSubsystem = new ShooterSubsystem();
		controlPanelSubsystem = new ControlPanelSubsystem();
		limelightSubsystem = new LimelightSubsystem();
		navXSubsystem = new NavXSubsystem();
		
		CommandScheduler.getInstance().registerSubsystem(
			driveTrainSubsystem,
			intakeSubsystem,
			shooterSubsystem,
			controlPanelSubsystem,
			limelightSubsystem,
			navXSubsystem
		);

		// Button Mappings
		/*
		getButton("Y")
			.whenPressed(new ToggleControlPanelSpinner(controlPanelSubsystem, true))
			.whenReleased(new ToggleControlPanelSpinner(controlPanelSubsystem, false));
		getButton("X")
			.whenPressed(new ToggleShooter(shooterSubsystem, true))
			.whenReleased(new ToggleShooter(shooterSubsystem, false));
		getButton("A")
			.whenPressed(new AutoSpinControlPanel(controlPanelSubsystem));
		getButton("B")
			.whenPressed(new ToggleIntake(intakeSubsystem));
		*/
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
		autonCommand = new Auton(driveTrainSubsystem, intakeSubsystem, autonChooser.getSelected()); // get chosen AutonType
		if(autonCommand != null) autonCommand.schedule(); // start auton
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		if(autonCommand != null) autonCommand.cancel(); // finish auton

		teleopCommand = new Teleop(xboxController, driveTrainSubsystem, intakeSubsystem); // overlaying teleop command for the teleop period
		teleopCommand.schedule(); // start teleop
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

	private JoystickButton getButton(String name) {
		return new JoystickButton(xboxController, XboxController.Button.valueOf("k"+name).value);
	}

	public static double getTimePeriod() {
		return period;
	}
}
