package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SolenoidChannels;

public class ClimbSubsystem extends SubsystemBase {

    private final DoubleSolenoid miniPistonL = new DoubleSolenoid(SolenoidChannels.CLIMB_MINI_L_FWD, SolenoidChannels.CLIMB_MINI_L_REV);
    private final DoubleSolenoid miniPistonR = new DoubleSolenoid(SolenoidChannels.CLIMB_MINI_R_FWD, SolenoidChannels.CLIMB_MINI_R_REV);

    private final DoubleSolenoid pistonL = new DoubleSolenoid(SolenoidChannels.CLIMB_L_FWD, SolenoidChannels.CLIMB_L_REV);
    private final DoubleSolenoid pistonR = new DoubleSolenoid(SolenoidChannels.CLIMB_R_FWD, SolenoidChannels.CLIMB_R_REV);

    public void setMiniPistons(Value direction) {
        miniPistonL.set(direction);
        miniPistonR.set(direction);
    }

	public void setPistons(Value direction) {
        pistonL.set(direction);
        pistonR.set(direction);
	}
}
