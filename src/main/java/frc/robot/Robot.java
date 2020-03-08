/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.wpilibj.XboxController.Button.*;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.commands.climb.*;
import frc.robot.commands.controlpanel.*;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.intake.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.main.*;
import frc.robot.commands.main.Auton.AutonType;
import frc.robot.commands.shooter.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimbSubsystem.ClimbDirection;

import static frc.robot.Constants.Shooter.*;

public class Robot extends TimedRobot {

    public static XboxController xboxController1;
    public static XboxController xboxController2;

    public static DriveTrainSubsystem driveTrainSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ControlPanelSubsystem controlPanelSubsystem;
    public static LimelightSubsystem limelightSubsystem;
    public static IndexerTowerSubsystem indexerTowerSubsystem;
    public static ClimbSubsystem climbSubsystem;

    private Auton autonCommand;

    private SendableChooser<AutonType> autonChooser;

    private DigitalOutput LEDOutput = new DigitalOutput(0);
    private Compressor compressor = new Compressor();

    public static double PERIOD = 0.02;

    @Override
    public void robotInit() {
        // auton chooser
        autonChooser = new SendableChooser<>();
        AutonType[] autonTypes = AutonType.values();
        for(AutonType autonType : autonTypes) {
            if(autonType == AutonType.SHOOT_AND_MOVE_FORWARD) {
                autonChooser.setDefaultOption(autonType.toString(), autonType);
            } else {
                autonChooser.addOption(autonType.toString(), autonType);
            }
        }
        SmartDashboard.putData(autonChooser);

        // subsystems
        xboxController1 = new XboxController(Controller.PORT_1);
        xboxController2 = new XboxController(Controller.PORT_2);
        driveTrainSubsystem = new DriveTrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        controlPanelSubsystem = new ControlPanelSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        indexerTowerSubsystem = new IndexerTowerSubsystem();
        climbSubsystem = new ClimbSubsystem();

        // button mappings
        getButton(xboxController1, kA).whenPressed(new ToggleClimb(ClimbDirection.DOWN));
        getButton(xboxController1, kB);
        getButton(xboxController1, kX).whenPressed(new AutoSpinControlPanel());
        getButton(xboxController1, kY).whenPressed(new ToggleClimb(ClimbDirection.UP));
        getButton(xboxController1, kBumperLeft).whenHeld(new JustMotor());
        getButton(xboxController1, kBumperRight).whenHeld(new ToggleIntake());

        getButton(xboxController2, kA).whenHeld(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_1));
        getButton(xboxController2, kB).whenHeld(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_2));
        getButton(xboxController2, kX).whenHeld(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_3));
        getButton(xboxController2, kY).whenHeld(new AutoAim());
        getButton(xboxController2, kBumperLeft).whenHeld(new ReverseTower());
        getButton(xboxController2, kBumperRight).whenHeld(new ToggleIndexerTower());

        // electrical
        LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);
        compressor.setClosedLoopControl(true);
        compressor.start();
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
        limelightSubsystem.toggleLight(false);
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void autonomousInit() {
        LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);
        limelightSubsystem.toggleLight(false);

        autonCommand = new Auton(autonChooser.getSelected());
        autonCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        if(autonCommand != null) autonCommand.cancel();

        CommandScheduler.getInstance().setDefaultCommand(driveTrainSubsystem, new TeleopDrive());
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

    private JoystickButton getButton(XboxController controller, XboxController.Button btn) {
        return new JoystickButton(controller, btn.value);
    }

}
