package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.DriveTrainSubsystem;

public class RotateDegrees extends ProfiledPIDCommand {

    // see these links:
    //  -  https://docs.wpilib.org/en/latest/docs/software/advanced-control/controllers/pidcontroller.html
    //  -  https://docs.wpilib.org/en/latest/docs/software/commandbased/profilepid-subsystems-commands.html

    // ProfiledPIDCommand takes in values through it's super constructor and handles the command's initialize, execute, and end functions

    public RotateDegrees(DriveTrainSubsystem driveTrainSubsystem, double degrees) {
		super(
            DriveTrain.PID_CONTROLLER, // PIDController with P, I, D, and Trapezoid constants
            driveTrainSubsystem::rotateDegreesGetMeasurement, // PID input supplier
            Math.abs(degrees), // PID goal
            driveTrainSubsystem::rotateDegreesUseOutput, // PID output consumer
            driveTrainSubsystem // required subsystems
        );
        driveTrainSubsystem.rotateDegreesInitialize(degrees);
    }

    @Override
	public boolean isFinished() {
		return getController().atGoal();
	}

}