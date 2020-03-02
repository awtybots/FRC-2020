package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriveTrain.*;

import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
import frc.robot.subsystems.DriveTrainSubsystem.MotorGroup;
import static frc.robot.Robot.*;

public class DriveInches extends CommandBase {

    private final double goalDistance;
    private double goalSpeed;
    private double currentDistance = 0;

    public DriveInches(double inches) {
        addRequirements(driveTrainSubsystem);
        this.goalDistance = inches;
        this.goalSpeed = Math.signum(goalDistance);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.resetSensors();
    }

    @Override
    public void execute() {
        currentDistance = driveTrainSubsystem.getWheelDistance(MotorGroup.ALL);

        // stopping distance (ramped velocity only)
        if(DRIVE_MODE == DriveMode.RAMPED_VELOCITY) {
            double currentVelocity = driveTrainSubsystem.getWheelVelocity(MotorGroup.ALL);
            double remainingDistance = Math.abs(goalDistance - currentDistance);
            double stoppingDistance = currentVelocity * currentVelocity / MAX_ACCELERATION / 2.0;
            if(stoppingDistance >= remainingDistance) {
                goalSpeed = 0; // start slowing down before we hit the target
            }
        }

        if(DRIVE_MODE == DriveMode.PERCENT || DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
            driveTrainSubsystem.setMotorOutput(goalSpeed * MAX_OUTPUT_AUTON, goalSpeed * MAX_OUTPUT_AUTON);
        } else {
            driveTrainSubsystem.setGoalVelocity(goalSpeed * MAX_VELOCITY_AUTON, goalSpeed * MAX_VELOCITY_AUTON);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentDistance) >= Math.abs(goalDistance);
    }

}