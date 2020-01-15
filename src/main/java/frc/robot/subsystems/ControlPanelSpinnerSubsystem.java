package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ControlPanelSpinner.*;
import frc.robot.Constants.MotorIDs;

public class ControlPanelSpinnerSubsystem extends SubsystemBase {

    private final WPI_TalonSRX spinner = new WPI_TalonSRX(MotorIDs.CONTROL_PANEL_SPINNER);

    private final Timer timer = new Timer();
    private double rotations;
    
    public ControlPanelSpinnerSubsystem() {
        spinner.configFactoryDefault();
        spinner.setNeutralMode(BRAKE_MODE);
        spinner.configSelectedFeedbackSensor(MOTOR_FEEDBACK_DEVICE);

        toggle(false);
    }
    @Override
    public void periodic() {
        double revsPerSecond = Math.abs(spinner.getSelectedSensorVelocity()) * 10.0 / ENCODER_UNITS;
        rotations += revsPerSecond * timer.get() * WHEEL_CIRCUMFERENCE / CONTROL_PANEL_CIRCUMFERENCE;
        timer.reset();
    }

    public void toggle(boolean on) {
        spinner.set(ControlMode.PercentOutput, on ? MOTOR_SPEED : 0);
    }

    public void resetRotations() {
        timer.reset();
        rotations = 0;
    }
    public double getRotations() {
        return rotations;
    }
}