package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class Climb extends InstantCommand {

    //private static ClimbDirection currentPosition = ClimbDirection.DOWN;

    private boolean pos;

    public Climb() {
        this.pos = false;
    }
    public Climb(boolean dir) {
        this.pos = dir;
    }

    @Override
    public void initialize() {
        if(pos == false){
            pos = true;
            climbSubsystem.setPistons(Value.kForward);
        } else {
            pos = false;
            climbSubsystem.setPistons(Value.kReverse);
        }
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