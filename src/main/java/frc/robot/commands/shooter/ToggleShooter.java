package frc.robot.commands.shooter;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Robot.*;

public class ToggleShooter extends CommandBase {

    public ToggleShooter() {
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setGoalFlywheelRevsPerSecond(SHOOTER_TELEOP_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setGoalFlywheelRevsPerSecond(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}