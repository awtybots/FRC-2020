package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;

public class SetShooterSpeed extends CommandBase {

    private double rps;

    public SetShooterSpeed(double rps) {
        this.rps = rps;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelGoalVelocity(rps);
    }

    @Override
    public void execute() {
        //indexerTowerSubsystem.toggle(shooterSubsystem.isVelocityAtGoal());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelGoalVelocity(0);
        indexerTowerSubsystem.toggle(false);
    }

}