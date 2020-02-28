package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SolenoidChannels;

public class ClimbSubsystem extends SubsystemBase {

    private final DoubleSolenoid pistons = new DoubleSolenoid(SolenoidChannels.CLIMB_FWD, SolenoidChannels.CLIMB_REV);

    public ClimbSubsystem() {
        setPistons(Value.kReverse);
    }

	public void setPistons(Value direction) {
        pistons.set(direction);
	}
}
