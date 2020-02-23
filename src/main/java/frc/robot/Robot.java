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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.wpilibj.XboxController.Button.*;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.commands.climb.*;
import frc.robot.commands.controlpanel.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.intake.MoveIntake.IntakePosition;
import frc.robot.commands.main.*;
import frc.robot.commands.main.Auton.AutonType;
import frc.robot.commands.shooter.*;
import frc.robot.commands.shooter.SpinTurret.TurretDirection;
import frc.robot.subsystems.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.DriveTrain.*;

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
    public static MusicSubsystem musicSubsystem;

    private Teleop teleopCommand;
    private Auton autonCommand;

    private SendableChooser<AutonType> autonChooser;

    private static double period;
    private static PowerDistributionPanel pdp;

    private DigitalOutput LEDOutput = new DigitalOutput(0);

    @Override
    public void robotInit() {
        // runtime constants
        period = getPeriod();
        pdp = new PowerDistributionPanel();

        // set LED color
        LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);

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
        xboxController1 = new XboxController(Controller.PORT_1);
        xboxController2 = new XboxController(Controller.PORT_2);
        driveTrainSubsystem = new DriveTrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        controlPanelSubsystem = new ControlPanelSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        indexerTowerSubsystem = new IndexerTowerSubsystem();
        climbSubsystem = new ClimbSubsystem();
        musicSubsystem = new MusicSubsystem();

        // button mappings
        getButton(xboxController1, kA).whenHeld(new ToggleIntake()).whenPressed(new MoveIntake(IntakePosition.DOWN)).whenReleased(new MoveIntake(IntakePosition.UP));
        getButton(xboxController1, kB).whenPressed(new AutoSpinControlPanel());
        getButton(xboxController1, kX);
        getButton(xboxController1, kY).whenPressed(new Climb());
        getButton(xboxController2, kBumperLeft);
        getButton(xboxController2, kBumperRight);

        getButton(xboxController2, kA).whenPressed(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_1)).whenReleased(new SetShooterSpeed(0));
        getButton(xboxController2, kB).whenPressed(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_2)).whenReleased(new SetShooterSpeed(0));
        getButton(xboxController2, kX).whenPressed(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_3)).whenReleased(new SetShooterSpeed(0));
        getButton(xboxController2, kY).whenHeld(new ToggleIndexerTower());
        getButton(xboxController2, kBumperLeft).whenHeld(new SpinTurret(TurretDirection.LEFT));
        getButton(xboxController2, kBumperRight).whenHeld(new SpinTurret(TurretDirection.RIGHT));
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
        driveTrainSubsystem.setDriveMode(AUTON_DRIVE_MODE);
        autonCommand = new Auton(autonChooser.getSelected()); // get chosen AutonType
        autonCommand.schedule(); // start auton
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        if(autonCommand != null) autonCommand.cancel(); // finish auton

        driveTrainSubsystem.setDriveMode(TELEOP_DRIVE_MODE);
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

    private JoystickButton getButton(XboxController controller, XboxController.Button btn) {
        return new JoystickButton(controller, btn.value);
    }

    public static double getLoopTime() {
        return period;
    }
    public static PowerDistributionPanel getPDP() {
        return pdp;
    }

}
