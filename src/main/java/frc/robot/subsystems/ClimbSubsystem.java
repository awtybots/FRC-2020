package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SolenoidChannels.*;

public class ClimbSubsystem extends SubsystemBase {

    private DoubleSolenoid anglePistons = new DoubleSolenoid(ANGLE_CLIMB_FWD, ANGLE_CLIMB_REV);
    private DoubleSolenoid pistons = new DoubleSolenoid(CLIMB_FWD, CLIMB_REV);

    public boolean anglePistonsUp = false;
    public boolean pistonsUp = false;

    public ClimbSubsystem() {
        angleClimb(false);
        climb(false);
    }

    public void angleClimb(boolean up) {
        anglePistonsUp = up;
        anglePistons.set(up ? Value.kForward : Value.kReverse);
    }

    public void climb(boolean up) {
        pistonsUp = up;
        pistons.set(up ? Value.kForward : Value.kReverse);
    }

}
