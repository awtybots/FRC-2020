package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveInches extends CommandBase {

    private DriveTrainSubsystem driveTrainSubsystem;
    private double inches;

    public DriveInches(DriveTrainSubsystem driveTrainSubsystem, double inches) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.inches = inches;
		addRequirements(driveTrainSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.driveInchesInitialize(inches);
    }

    @Override
    public void execute() {
        driveTrainSubsystem.driveInchesExecute();
    }

    @Override
	public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
	public boolean isFinished() {
		return driveTrainSubsystem.driveInchesIsFinished();
	}

}