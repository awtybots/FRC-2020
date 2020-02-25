package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class Climb extends InstantCommand {

    private static ClimbDirection currentPosition = ClimbDirection.DOWN;

    private ClimbDirection pos;

    public Climb() {
        this.pos = null;
    }
    public Climb(ClimbDirection dir) {
        this.pos = dir;
    }

    @Override
    public void initialize() {
        if(pos == null) pos = currentPosition.getOpposite();
        currentPosition = pos;
        climbSubsystem.setPistons(pos.getDirection());
    }

    public enum ClimbDirection {
        UP  (DoubleSolenoid.Value.kForward),
        DOWN(DoubleSolenoid.Value.kReverse);

        private DoubleSolenoid.Value direction;
        private ClimbDirection(DoubleSolenoid.Value direction) {
            this.direction = direction;
        }

        public DoubleSolenoid.Value getDirection() {
            return direction;
        }
        public ClimbDirection getOpposite() {
            switch(this) {
                case UP: return DOWN;
                case DOWN: return UP;
                default: return DOWN;
            }
        }
    }
}