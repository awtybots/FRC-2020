package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelSpinner;
import frc.robot.Constants.MotorIDs;

public class ControlPanelSpinnerSubsystem extends SubsystemBase {

    private final WPI_TalonSRX spinner = new WPI_TalonSRX(MotorIDs.CONTROL_PANEL_SPINNER);
    
    public ControlPanelSpinnerSubsystem() {
        stop();
    }

    public void start() {
        spinner.set(ControlMode.PercentOutput, ControlPanelSpinner.MOTOR_SPEED);
    }

    public void stop() {
        spinner.set(ControlMode.PercentOutput, 0);
    }
}