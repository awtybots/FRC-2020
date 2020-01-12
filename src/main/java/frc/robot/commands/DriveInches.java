package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveInches extends ProfiledPIDCommand {

    private DriveTrainSubsystem driveTrainSubsystem;

    public DriveInches(DriveTrainSubsystem driveTrainSubsystem, double inches) {
        super(DriveTrain.PID_CONTROLLER, driveTrainSubsystem::driveInchesGetMeasurement, inches, driveTrainSubsystem::driveInchesUseOutput, driveTrainSubsystem);
        this.driveTrainSubsystem = driveTrainSubsystem;
        driveTrainSubsystem.driveInchesInitialize(inches);
    }

    @Override
	public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
	public boolean isFinished() {
		return getController().atGoal();
	}

}