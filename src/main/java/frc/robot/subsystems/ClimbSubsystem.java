package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.RobotMap.SolenoidChannels.*;

public class ClimbSubsystem extends SubsystemBase {

    private DoubleSolenoid pistons = new DoubleSolenoid(CLIMB_FWD, CLIMB_REV);

    public boolean anglePistonsUp = false;
    public boolean pistonsUp = false;

    public ClimbSubsystem() {
        toggleClimb(false);
    }

    public void toggleClimb(boolean up) {
        pistonsUp = up;
        pistons.set(up ? Value.kForward : Value.kReverse);
    }

}
