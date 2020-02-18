package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;
import static frc.robot.Constants.Shooter.*;

public class AngleClimber extends CommandBase {

    private static ClimberAngle currentPosition;

    private ClimberAngle pos;

    public AngleClimber() {
        this(null);
    }
    public AngleClimber(ClimberAngle dir) {
        this.pos = dir;
        addRequirements(shooterSubsystem); // cancel any other commands changing the turret position
    }

    @Override
    public void initialize() {
        if(pos == null) pos = currentPosition.getOpposite();
        if(pos == ClimberAngle.UP) shooterSubsystem.setTurretGoalAngle(TURRET_CLIMB_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.setPistons(pos.getDirection());
        currentPosition = pos;
    }

    @Override
    public boolean isFinished() {
        return (pos == ClimberAngle.DOWN) || Math.abs(shooterSubsystem.getTurretAngle() - TURRET_CLIMB_ANGLE) < TURRET_CLIMB_ANGLE_TOLERANCE;
    }

    public enum ClimberAngle {
        UP  (DoubleSolenoid.Value.kForward),
        DOWN(DoubleSolenoid.Value.kReverse);

        private DoubleSolenoid.Value direction;
        private ClimberAngle(DoubleSolenoid.Value direction) {
            this.direction = direction;
        }

        public DoubleSolenoid.Value getDirection() {
            return direction;
        }
        public ClimberAngle getOpposite() {
            switch(this) {
                case UP: return DOWN;
                case DOWN: return UP;
                default: return null;
            }
        }
    }
}