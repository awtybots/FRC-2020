package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriveTrain.*;

import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
import static frc.robot.Robot.*;

public class RotateDegrees extends CommandBase {

    private double goalRotation;
    private double goalVelocity;
    private double currentRotation = 0;

    public RotateDegrees(double degrees) {
        addRequirements(driveTrainSubsystem);
        goalRotation = -degrees;
        goalVelocity = Math.signum(-degrees) * MAX_VELOCITY;
    }

    @Override
    public void initialize() {
        goalRotation = Math.floorMod((int)(driveTrainSubsystem.getRotation() + goalRotation), 360);
        driveTrainSubsystem.resetEncoders();
    }

    @Override
    public void execute() {
        // rotation values
        currentRotation = driveTrainSubsystem.getRotation();

        // stopping distance
        if(AUTON_DRIVE_MODE == DriveMode.TRAPEZOIDAL_VELOCITY) {
            double remainingDistance = goalRotation - currentRotation;
            if(ROTATE_DEGREES_SLOW_THRESHOLD >= remainingDistance) {
                goalVelocity = 0; // start slowing down before we hit the target
            }
        }

        // motors
        if(AUTON_DRIVE_MODE == DriveMode.DIRECT) {
            driveTrainSubsystem.setMotorOutput(goalVelocity/MAX_VELOCITY, -goalVelocity/MAX_VELOCITY);
        } else {
            driveTrainSubsystem.setGoalVelocity(goalVelocity, -goalVelocity);
        }

        // SmartDashbaord
        SmartDashboard.putNumber("RD - Current Distance", currentRotation);
        SmartDashboard.putNumber("RD - Goal Distance", goalRotation);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentRotation - goalRotation) <= ROTATE_DEGREES_GOAL_TOLERANCE;
    }

}