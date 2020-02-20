package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.DriveTrain.*;

import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
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
        double currentVelocity = driveTrainSubsystem.getWheelVelocity(MotorGroup.ALL);
        currentDistance = driveTrainSubsystem.getWheelDistance(MotorGroup.ALL);

        // stopping distance
        if(AUTON_DRIVE_MODE == DriveMode.TRAPEZOIDAL_VELOCITY) {
            double remainingDistance = Math.abs(goalDistance - currentDistance);
            double stoppingDistance = currentVelocity * currentVelocity / MAX_ACCELERATION / 2.0;
            SmartDashboard.putNumber("DI - Stopping Distance", stoppingDistance);
            if(stoppingDistance >= remainingDistance) {
                goalVelocity = 0; // start slowing down before we hit the target
            }
        }

        // motors
        driveTrainSubsystem.setGoalVelocity(goalVelocity, goalVelocity);

        // SmartDashbaord
        SmartDashboard.putNumber("DI - Current Distance", currentDistance);
        SmartDashboard.putNumber("DI - Goal Distance", goalDistance);
        SmartDashboard.putNumber("DI - Current Velocity", currentVelocity);
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