package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class AngleClimber extends InstantCommand {

    private static ClimberAngle currentPosition;

    private ClimberAngle pos;

    public AngleClimber() {
        this.pos = null;
    }
    public AngleClimber(ClimberAngle dir) {
        this.pos = dir;
    }

    @Override
    public void initialize() {
        if(pos == null) pos = currentPosition.getOpposite();
        currentPosition = pos;
        climbSubsystem.setPistons(pos.getDirection());
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