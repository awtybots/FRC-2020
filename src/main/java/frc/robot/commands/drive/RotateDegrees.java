package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriveTrain.*;
import frc.robot.subsystems.DriveTrainSubsystem.MotorGroup;
import static frc.robot.Robot.*;

public class RotateDegrees extends CommandBase {

    private final double goalDistance;
    private double goalVelocity;
    private double currentDistance = 0;
    private double multiplier;

    public RotateDegrees(double degrees) {
        addRequirements(driveTrainSubsystem);
        this.goalDistance = Math.abs(degrees)/360 * ROBOT_CIRMCUMFERENCE;
        this.multiplier = Math.signum(degrees);
        this.goalVelocity = multiplier * MAX_VELOCITY;
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.resetDistance();
    }

    @Override
    public void execute() {
        // encoders
        double currentVelocity = driveTrainSubsystem.getVelocity(MotorGroup.ALL, true);
        currentDistance = driveTrainSubsystem.getDistance(true);

        // stopping distance
        double remainingDistance = goalDistance - currentDistance;
        double stoppingDistance = currentVelocity * currentVelocity / MAX_ACCELERATION / 2;
        if(stoppingDistance >= remainingDistance) {
            goalVelocity = 0; // start slowing down before we hit the target
        }

        // motors
        driveTrainSubsystem.setGoalVelocity(goalVelocity * multiplier, goalVelocity * -multiplier);

        // SmartDashbaord
        SmartDashboard.putNumber("RD - Current Distance", currentDistance);
        SmartDashboard.putNumber("RD - Goal Distance", goalDistance);
        SmartDashboard.putNumber("RD - Current Velocity", currentVelocity);
        SmartDashboard.putNumber("RD - Goal Velocity", goalVelocity);
        SmartDashboard.putNumber("RD - Stopping Distance", stoppingDistance);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.smoothStop();
    }

    @Override
	public boolean isFinished() {
		return currentDistance >= goalDistance - GOAL_TOLERANCE;
	}

}