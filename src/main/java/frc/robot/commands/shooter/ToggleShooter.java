package frc.robot.commands.shooter;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooter extends InstantCommand {

    private final ShooterSubsystem shooter;
    private final boolean on;

    public ToggleShooter(ShooterSubsystem shooter, boolean on) {
        addRequirements(shooter);
        this.shooter = shooter;
        this.on = on;
    }

    @Override
    public void initialize() {
        shooter.setGoalFlywheelRevsPerSecond(on ? SHOOTER_TELEOP_SPEED : 0);
    }

}