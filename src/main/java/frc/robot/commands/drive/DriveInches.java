package frc.robot.commands.drive;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;
import frc.robot.subsystems.DrivetrainSubsystem.MotorGroup;

public class DriveInches extends CommandBase {
  private final double goalDistance;
  private double goalSpeed;
  private double currentDistance = 0;

  public DriveInches(double inches) {
    addRequirements(drivetrainSubsystem);
    this.goalDistance = inches;
    this.goalSpeed = Math.signum(goalDistance);
  }

  @Override
  public void initialize() {
    drivetrainSubsystem.resetSensors();
  }

  @Override
  public void execute() {
    currentDistance = MotorGroup.ALL.getWheelDistance();

    if (DRIVE_MODE == DriveMode.PERCENT || DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
      drivetrainSubsystem.setMotorOutput(
          goalSpeed * MAX_OUTPUT_AUTON, goalSpeed * MAX_OUTPUT_AUTON);
    } else {
      drivetrainSubsystem.setGoalVelocity(
          goalSpeed * MAX_VELOCITY_AUTON, goalSpeed * MAX_VELOCITY_AUTON);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(currentDistance) >= Math.abs(goalDistance);
  }
}
