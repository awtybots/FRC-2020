package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriveTrain.*;
import frc.robot.subsystems.DriveTrainSubsystem.MotorGroup;
import static frc.robot.Robot.*;

public class DriveInches extends CommandBase {

    private final double goalDistance;
    private double goalVelocity;
    private double currentDistance = 0;

    public DriveInches(double inches) {
        addRequirements(driveTrainSubsystem);
        this.goalDistance = inches;
        this.goalVelocity = Math.signum(goalDistance) * MAX_VELOCITY;
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.resetEncoders();
    }

    @Override
    public void execute() {
        // encoders
        double currentVelocity = driveTrainSubsystem.getWheelVelocity(MotorGroup.ALL, false);
        currentDistance = driveTrainSubsystem.getDistance(false);

        // stopping distance
        double remainingDistance = Math.abs(goalDistance - currentDistance);
        double stoppingDistance = currentVelocity * currentVelocity / MAX_ACCELERATION / 2;
        if(stoppingDistance >= remainingDistance) {
            goalVelocity = 0; // start slowing down before we hit the target
        }

        // motors
        driveTrainSubsystem.setGoalVelocity(goalVelocity, goalVelocity);

        // SmartDashbaord
        SmartDashboard.putNumber("DI - Current Distance", currentDistance);
        SmartDashboard.putNumber("DI - Goal Distance", goalDistance);
        SmartDashboard.putNumber("DI - Current Velocity", currentVelocity);
        SmartDashboard.putNumber("DI - Goal Velocity", goalVelocity);
        SmartDashboard.putNumber("DI - Stopping Distance", stoppingDistance);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.smoothStop();
    }

    @Override
	public boolean isFinished() {
		return Math.abs(currentDistance) >= Math.abs(goalDistance) - GOAL_TOLERANCE;
	}

}