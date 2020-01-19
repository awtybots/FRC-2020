package frc.robot.commands.shooter;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooter extends CommandBase {

    private final ShooterSubsystem shooter;

    public ToggleShooter(ShooterSubsystem shooter) {
        addRequirements(shooter);
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setGoalFlywheelRevsPerSecond(SHOOTER_TELEOP_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setGoalFlywheelRevsPerSecond(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}