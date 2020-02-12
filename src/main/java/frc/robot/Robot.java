/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.controlpanel.AutoSpinControlPanel;
import frc.robot.commands.intake.ToggleIndexerTower;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.main.*;
import frc.robot.commands.main.Auton.AutonType;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ToggleShooter;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {

    public static XboxController xboxController1;

    public static DriveTrainSubsystem driveTrainSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ControlPanelSubsystem controlPanelSubsystem;
    public static LimelightSubsystem limelightSubsystem;
    public static IndexerTowerSubsystem indexerTowerSubsystem;

    private Teleop teleopCommand;
    private Auton autonCommand;

    private SendableChooser<AutonType> autonChooser;

    private static double period;
    private static PowerDistributionPanel pdp;

    @Override
    public void robotInit() {
        // runtime constants
        period = getPeriod();
        pdp = new PowerDistributionPanel();

        // auton chooser
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

        // subsystems
        xboxController1 = new XboxController(Controller.PORT);
        driveTrainSubsystem = new DriveTrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        controlPanelSubsystem = new ControlPanelSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        indexerTowerSubsystem = new IndexerTowerSubsystem();

        // button mappings
        getButton("A").whenHeld(new ToggleShooter());
        getButton("B").whenHeld(new AutoShoot());
        getButton("X").whenPressed(new AutoSpinControlPanel());
        //getButton("Y")
        getButton("BumperLeft").whenHeld(new ToggleIndexerTower());
        getButton("BumperRight").whenHeld(new ToggleIntake());
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
        autonCommand = new Auton(autonChooser.getSelected()); // get chosen AutonType
        autonCommand.schedule(); // start auton
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        if(autonCommand != null) autonCommand.cancel(); // finish auton

        teleopCommand = new Teleop(); // overlaying teleop command for the teleop period
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
        return new JoystickButton(xboxController1, XboxController.Button.valueOf("k"+name).value);
    }

    public static double getLoopTime() {
        return period;
    }
    public static PowerDistributionPanel getPDP() {
        return pdp;
    }

}
