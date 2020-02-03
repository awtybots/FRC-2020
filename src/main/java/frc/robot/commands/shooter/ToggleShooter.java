package frc.robot.commands.shooter;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Robot.*;

public class ToggleShooter extends CommandBase {

    public ToggleShooter() {
        addRequirements(shooterSubsystem);
        SmartDashboard.setDefaultNumber("Shooter Goal RPM", FLYWHEEL_TELEOP_SPEED * 60.0); // TODO
    }

    @Override
    public void initialize() {
        shooterSubsystem.setGoalFlywheelRevsPerSecond(SmartDashboard.getNumber("Shooter Goal RPM", FLYWHEEL_TELEOP_SPEED * 60.0) / 60.0);
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