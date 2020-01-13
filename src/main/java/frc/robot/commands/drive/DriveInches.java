package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveInches extends ProfiledPIDCommand {

    private final DriveTrainSubsystem driveTrainSubsystem;

    public DriveInches(DriveTrainSubsystem driveTrainSubsystem, double inches) {
        super(
            DriveTrain.PID_CONTROLLER, // PIDController with P, I, D, and Trapezoid constants
            driveTrainSubsystem::driveInchesGetMeasurement, // PID input supplier
            Math.abs(inches), // PID goal
            driveTrainSubsystem::driveInchesUseOutput, // PID output consumer
            driveTrainSubsystem // requirements
        );
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