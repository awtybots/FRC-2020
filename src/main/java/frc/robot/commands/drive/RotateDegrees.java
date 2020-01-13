package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RotateDegrees extends ProfiledPIDCommand {

    private final DriveTrainSubsystem driveTrainSubsystem;

    public RotateDegrees(DriveTrainSubsystem driveTrainSubsystem, double degrees) {
		super(
            DriveTrain.PID_CONTROLLER, // PIDController with P, I, D, and Trapezoid constants
            driveTrainSubsystem::rotateDegreesGetMeasurement, // PID input supplier
            Math.abs(degrees), // PID goal
            driveTrainSubsystem::rotateDegreesUseOutput, // PID output consumer
            driveTrainSubsystem // requirements
        );
        this.driveTrainSubsystem = driveTrainSubsystem;
        driveTrainSubsystem.rotateDegreesInitialize(degrees);
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