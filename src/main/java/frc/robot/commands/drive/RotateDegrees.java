package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriveTrain.*;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem.MotorGroup;

public class RotateDegrees extends CommandBase {

    private final DriveTrainSubsystem driveTrainSubsystem;

    private final double goalDistance;
    private double goalVelocity;
    private double currentDistance = 0;
    private double multiplier;

    private final Timer timer = new Timer();

    public RotateDegrees(DriveTrainSubsystem driveTrainSubsystem, double degrees) {
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.goalDistance = Math.abs(degrees)/360 * ROBOT_CIRMCUMFERENCE;
        this.multiplier = Math.signum(degrees);
        this.goalVelocity = multiplier * MAX_VELOCITY;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        // elapsed time
        double elapsedTime = timer.get();
        timer.reset();

        // encoders
        double currentVelocity = driveTrainSubsystem.getAverageInchesPerSecond(MotorGroup.ALL, true);
        currentDistance += currentVelocity * elapsedTime;

        // stopping distance
        double remainingDistance = goalDistance - currentDistance;
        double stoppingDistance = currentVelocity * currentVelocity / MAX_ACCELERATION / 2;
        if(stoppingDistance >= remainingDistance) {
            goalVelocity = 0; // start slowing down before we hit the target
        }

        // motors
        driveTrainSubsystem.setGoalVelocity(goalVelocity * multiplier, goalVelocity * -multiplier);

        // SmartDashbaord
        SmartDashboard.putNumber("Current Distance", currentDistance);
        SmartDashboard.putNumber("Goal Distance", goalDistance);
        SmartDashboard.putNumber("Current Velocity", currentVelocity);
        SmartDashboard.putNumber("Goal Velocity", goalVelocity);
        SmartDashboard.putNumber("Stopping Distance", stoppingDistance);
    }

    @Override
	public boolean isFinished() {
		return currentDistance >= goalDistance - GOAL_TOLERANCE;
	}

}