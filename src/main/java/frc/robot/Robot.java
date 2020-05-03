package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.*;
import frc.robot.commands.main.*;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.main.Auton.AutonType;

public class Robot extends TimedRobot {

    public static DrivetrainSubsystem drivetrainSubsystem;
    public static IntakeSubsystem intakeSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static ControlPanelSubsystem controlPanelSubsystem;
    public static LimelightSubsystem limelightSubsystem;
    public static IndexerTowerSubsystem indexerTowerSubsystem;
    public static ClimbSubsystem climbSubsystem;
    public static OI oi;

    private Auton autonCommand;
    private SendableChooser<AutonType> autonChooser;

    private DigitalOutput LEDOutput = new DigitalOutput(0);
    private Compressor compressor = new Compressor();

    public static double PERIOD = 0.02;

    @Override
    public void robotInit() {
        // subsystems
        drivetrainSubsystem = new DrivetrainSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        controlPanelSubsystem = new ControlPanelSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        indexerTowerSubsystem = new IndexerTowerSubsystem();
        climbSubsystem = new ClimbSubsystem();
        oi = new OI();

        // electrical
        LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);
        compressor.setClosedLoopControl(true);
        compressor.start();

        // Put all Auton choices on SmartDashboard
        autonChooser = new SendableChooser<>();
        for(AutonType autonType : AutonType.values()) {
            if(autonType == AutonType.SHOOT_AND_MOVE_FORWARD) {
                autonChooser.setDefaultOption(autonType.toString(), autonType);
            } else {
                autonChooser.addOption(autonType.toString(), autonType);
            }
        }
        SmartDashboard.putData(autonChooser);
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
    public void robotPeriodic() { CommandScheduler.getInstance().run(); }

    @Override
    public void disabledInit() {
        limelightSubsystem.toggleLight(false);
    }

    @Override
    public void autonomousInit() {
        LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);
        limelightSubsystem.toggleLight(false);

        autonCommand = new Auton(autonChooser.getSelected());
        autonCommand.schedule();
    }

    @Override
    public void teleopInit() {
        if(autonCommand != null) autonCommand.cancel();

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new TeleopDrive());
    }
}
