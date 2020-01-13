package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelSpinner;
import frc.robot.Constants.MotorIDs;

public class ControlPanelSpinnerSubsystem extends SubsystemBase {

    private final WPI_TalonSRX spinner = new WPI_TalonSRX(MotorIDs.CONTROL_PANEL_SPINNER);
    
    public ControlPanelSpinnerSubsystem() {
        toggle(false);
    }

    public void toggle(boolean on) {
        spinner.set(ControlMode.PercentOutput, on ? ControlPanelSpinner.MOTOR_SPEED : 0);
    }
}