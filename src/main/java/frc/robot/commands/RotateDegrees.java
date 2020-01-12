package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RotateDegrees extends CommandBase {

    private DriveTrainSubsystem driveTrainSubsystem;
    private double degrees;

    public RotateDegrees(DriveTrainSubsystem driveTrainSubsystem, double degrees) {
		addRequirements(driveTrainSubsystem);
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.degrees = degrees;
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.rotateDegreesInitialize(degrees);
    }

    @Override
    public void execute() {
        driveTrainSubsystem.rotateDegreesExecute();
    }

    @Override
	public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
	public boolean isFinished() {
		return driveTrainSubsystem.rotateDegreesIsFinished();
	}

}