package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.shooterSubsystem;
import static frc.robot.Robot.indexerTowerSubsystem;

public class SetShooterSpeed extends CommandBase {

    private double rps;
    private boolean toggled;
    private boolean autoIndex;

    public SetShooterSpeed(double rpm) {
        this(rpm, false);
    }

    public SetShooterSpeed(double rpm, boolean autoIndex) {
        this.rps = rpm/60.0;
        this.autoIndex = autoIndex;
        toggled = false;
    }

    @Override
    public void initialize() {
        shooterSubsystem.setFlywheelGoalVelocity(rps);
    }

    @Override
    public void execute() {
        shooterSubsystem.flywheelPID();
        if(autoIndex && (!toggled) && shooterSubsystem.isVelocityAtGoal()) {
            indexerTowerSubsystem.toggle(true);
            toggled = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelGoalVelocity(0);
        if(toggled)
            indexerTowerSubsystem.toggle(false);
    }

}
