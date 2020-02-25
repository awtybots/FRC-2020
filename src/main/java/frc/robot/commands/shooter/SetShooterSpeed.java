package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;

public class SetShooterSpeed extends CommandBase {

    private double rps;
    private boolean toggled;

    public SetShooterSpeed(double rps) {
        this.rps = rps;
        toggled = false;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelGoalVelocity(rps);
    }

    @Override
    public void execute() {
        if((!toggled) && shooterSubsystem.isVelocityAtGoal()) {
            indexerTowerSubsystem.toggle(true);
            toggled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelGoalVelocity(0);
        if(toggled) indexerTowerSubsystem.toggle(false);
    }

}