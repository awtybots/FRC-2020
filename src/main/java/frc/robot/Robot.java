package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.main.*;
import frc.robot.commands.main.Auton.AutonType;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
  /// ----- Subsytems ----- ///
  public static DrivetrainSubsystem drivetrainSubsystem;
  public static IntakeSubsystem intakeSubsystem;
  public static ShooterSubsystem shooterSubsystem;
  public static ControlPanelSubsystem controlPanelSubsystem;
  public static LimelightSubsystem limelightSubsystem;
  public static IndexerTowerSubsystem indexerTowerSubsystem;
  public static ClimbSubsystem climbSubsystem;
  public static OI oi;

  /// ----- Autonomous ----- ///
  private Auton autonCommand;
  private SendableChooser<AutonType> autonChooser;

  /// ----- Electrical/CANBus ----- ///
  private DigitalOutput LEDOutput = new DigitalOutput(0);
  private Compressor compressor = new Compressor();

  @Override
  public void robotInit() {
    /// ---- Subsystems ---- ///
    drivetrainSubsystem = new DrivetrainSubsystem();
    limelightSubsystem = new LimelightSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    climbSubsystem = new ClimbSubsystem();
    controlPanelSubsystem = new ControlPanelSubsystem();
    indexerTowerSubsystem = new IndexerTowerSubsystem();
    oi = new OI();

    /// ---- Electrical ---- ///
    LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);
    compressor.setClosedLoopControl(true);

    // Add all Autons to SmartDashboard
    autonChooser = new SendableChooser<>();
    for (AutonType autonType : AutonType.values()) {
      if (autonType == AutonType.SHOOT_AND_MOVE_FORWARD) {
        autonChooser.setDefaultOption(autonType.toString(), autonType);
      } else {
        autonChooser.addOption(autonType.toString(), autonType);
      }
    }
    SmartDashboard.putData(autonChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    limelightSubsystem.toggleLight(false); // Plz don't blind us
  }

  @Override
  public void autonomousInit() {
    autonCommand = new Auton(autonChooser.getSelected());
    autonCommand.schedule();

    LEDOutput.set(DriverStation.getInstance().getAlliance() == Alliance.Red);
    limelightSubsystem.toggleLight(false);
  }

  @Override
  public void teleopInit() {
    if (autonCommand != null) autonCommand.cancel();

    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new TeleopDrive());
  }
}
