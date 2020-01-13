package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonSRX motor = new WPI_TalonSRX(MotorIDs.INTAKE);
    
    public IntakeSubsystem() {
        toggle(false);
    }

    public void toggle(boolean on) {
        motor.set(ControlMode.PercentOutput, on ? Intake.MOTOR_SPEED : 0);
    }
}