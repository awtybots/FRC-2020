package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class SetShooterSpeed extends InstantCommand {

    private double rps;

    public SetShooterSpeed(double rps) {
        addRequirements(shooterSubsystem);
        this.rps = rps;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelGoalVelocity(rps);
    }

}