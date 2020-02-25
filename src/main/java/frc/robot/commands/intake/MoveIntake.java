package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class MoveIntake extends InstantCommand {

    private static IntakePosition currentPosition;

    private IntakePosition pos;

    public MoveIntake() {
        this.pos = null;
    }
    public MoveIntake(IntakePosition pos) {
        this.pos = pos;
    }

    @Override
    public void initialize() {
        if(pos == null) pos = currentPosition.getOpposite();
        currentPosition = pos;
        intakeSubsystem.setPistons(pos.getDirection());
    }

    public enum IntakePosition {
        UP  (DoubleSolenoid.Value.kReverse),
        DOWN(DoubleSolenoid.Value.kForward);

        private DoubleSolenoid.Value direction;
        private IntakePosition(DoubleSolenoid.Value direction) {
            this.direction = direction;
        }

        public DoubleSolenoid.Value getDirection() {
            return direction;
        }
        public IntakePosition getOpposite() {
            switch(this) {
                case UP: return DOWN;
                case DOWN: return UP;
                default: return DOWN;
            }
        }
    }
}