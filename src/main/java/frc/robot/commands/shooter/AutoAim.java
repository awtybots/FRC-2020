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
        addRequirements(drivetrainSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() { // TODO error handling
        limelightSubsystem.setPipeline(Pipeline.POWER_PORT);
        limelightSubsystem.toggleLight(true);
    }

    @Override
    public void execute() {
        Vector3 limelightData = limelightSubsystem.getTargetData();
        if(limelightData == null) {
            drivetrainSubsystem.stop();
            System.err.println("Cannot auto-aim without a target!");
            return;
        }

        onTarget = Math.abs(limelightData.x) < TARGET_THRESHOLD_DEG;

        if(onTarget) {
            drivetrainSubsystem.stop();
        } else {
            double speed = MathUtil.clamp(limelightData.x / TARGET_SLOW_THRESHOLD_DEG, -1.0, 1.0);
            drivetrainSubsystem.setMotorOutput(speed * MAX_AIMING_DRIVE_OUTPUT, -speed * MAX_AIMING_DRIVE_OUTPUT);
        }
    }

    @Override
    public boolean isFinished() {
        return onTarget;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
        limelightSubsystem.toggleLight(false); // TODO error handling/warnings
    }

}
