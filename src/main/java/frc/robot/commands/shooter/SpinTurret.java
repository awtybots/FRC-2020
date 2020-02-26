package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;
import static frc.robot.Constants.Shooter.*;

public class SpinTurret extends CommandBase {

    private double direction;

    public SpinTurret(TurretDirection dir) {
        addRequirements(shooterSubsystem);
        this.direction = dir.getDirection();
    }

    @Override
    public void execute() {
        shooterSubsystem.setTurretSpeed(direction * TURRET_MAX_SPEED, true);
    }
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTurretSpeed(0, false);
    }

    public enum TurretDirection {
        RIGHT(1.0),
        LEFT(-1.0);

        private double direction;
        private TurretDirection(double direction) {
            this.direction = direction;
        }

        public double getDirection() {
            return direction;
        }
    }
}