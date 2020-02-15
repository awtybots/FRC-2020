package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.SolenoidChannels;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX motor = new WPI_TalonSRX(MotorIDs.INTAKE);
    private final DoubleSolenoid pistonL = new DoubleSolenoid(SolenoidChannels.INTAKE_L_FWD, SolenoidChannels.INTAKE_L_REV);
    private final DoubleSolenoid pistonR = new DoubleSolenoid(SolenoidChannels.INTAKE_R_FWD, SolenoidChannels.INTAKE_R_REV);

    public IntakeSubsystem() {
        toggle(false);
    }

    public void toggle(boolean on) {
        motor.set(ControlMode.PercentOutput, on ? MOTOR_SPEED : 0);
    }

	public void setPistons(Value direction) {
        pistonL.set(direction);
        pistonR.set(direction);
	}
}
