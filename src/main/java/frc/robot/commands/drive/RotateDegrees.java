package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.DriveTrain.*;

import static frc.robot.subsystems.DrivetrainSubsystem.DriveMode;
import static frc.robot.Robot.drivetrainSubsystem;

public class RotateDegrees extends CommandBase {

    private double currentRotation;
    private double goalRotation;
    private double goalSpeed;

    public RotateDegrees(double degrees) {
        addRequirements(drivetrainSubsystem);
        goalRotation = degrees;
        goalSpeed = Math.signum(degrees);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentRotation = drivetrainSubsystem.getRotation();
        goalSpeed = MathUtil.clamp((goalRotation - currentRotation) / ROTATE_DEGREES_SLOW_THRESHOLD, -1.0, 1.0);

        // motors
        if(DRIVE_MODE == DriveMode.PERCENT || DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
            drivetrainSubsystem.setMotorOutput(goalSpeed * MAX_OUTPUT_AUTON, -goalSpeed * MAX_OUTPUT_AUTON);
        } else {
            drivetrainSubsystem.setGoalVelocity(goalSpeed * MAX_VELOCITY_AUTON, -goalSpeed * MAX_VELOCITY_AUTON);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentRotation) >= Math.abs(goalRotation);
    }

}
