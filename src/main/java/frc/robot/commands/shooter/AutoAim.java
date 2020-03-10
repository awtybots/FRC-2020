package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.LimelightSubsystem.Pipeline;
import frc.robot.util.Vector3;

import static frc.robot.Robot.*;
import static frc.robot.Constants.Shooter.*;

public class AutoAim extends CommandBase {

    private boolean onTarget = false;

    public AutoAim() {
        addRequirements(driveTrainSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(Pipeline.POWER_PORT);
        limelightSubsystem.toggleLight(true);
    }

    @Override
    public void execute() {
        Vector3 limelightData = limelightSubsystem.getTargetData();
        if(limelightData == null) {
            driveTrainSubsystem.stop();
            System.err.println("Cannot auto-aim without a target!");
            return;
        }

        onTarget = Math.abs(limelightData.x) < TARGET_ANGLE_THRESHOLD;

        if(onTarget) {
            driveTrainSubsystem.stop();
        } else {
            double speed = MathUtil.clamp(limelightData.x / TARGET_ANGLE_SLOW_THRESHOLD, -1.0, 1.0);
            driveTrainSubsystem.setMotorOutput(speed * MAX_AIMING_DRIVE_OUTPUT, -speed * MAX_AIMING_DRIVE_OUTPUT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
        limelightSubsystem.toggleLight(false);
    }

    @Override
    public boolean isFinished() {
        return onTarget;
    }

}