package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem.Pipeline;
import frc.robot.util.Vector3;

public class AutoShoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;

    public AutoShoot(ShooterSubsystem shooter, LimelightSubsystem limelight) {
        addRequirements(shooter, limelight);
        this.shooter = shooter;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        limelight.setPipeline(Pipeline.POWER_PORT);
    }

    @Override
    public void execute() {
        shooter.aimTowards(limelight.getTargetVector(), new Vector3()); // use NavX velocity
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setGoalVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}