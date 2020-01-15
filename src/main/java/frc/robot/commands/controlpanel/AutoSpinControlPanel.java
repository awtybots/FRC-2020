package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.ControlPanelSpinner.*;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem.PanelColor;
import frc.robot.subsystems.ControlPanelSpinnerSubsystem;

public class AutoSpinControlPanel extends CommandBase {

    private final ControlPanelSpinnerSubsystem spinner;
    private final ColorSensorSubsystem sensor;

    private ControlType controlType;

    private PanelColor goalColor;

    public AutoSpinControlPanel(ControlPanelSpinnerSubsystem spinner, ColorSensorSubsystem sensor) {
        addRequirements(spinner, sensor);
        this.spinner = spinner;
        this.sensor = sensor;
    }

    @Override
    public void initialize() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
            controlType = ControlType.POSITION_CONTROL;
            goalColor = PanelColor.fromChar(gameData.charAt(0));
        } else {
            spinner.resetRotations();
        }

        spinner.toggle(true);
    }

    @Override
    public void execute() {
        if(controlType == ControlType.ROTATION_CONTROL) SmartDashboard.putNumber("Rotations", spinner.getRotations());
    }

    @Override
    public void end(boolean interrupted) {
        spinner.toggle(false);
    }

    @Override
    public boolean isFinished() {
        return (controlType == ControlType.ROTATION_CONTROL) ? (spinner.getRotations() >= ROTATIONS) : (sensor.getCurrentColor() == goalColor);
    }

    private enum ControlType {
        POSITION_CONTROL,
        ROTATION_CONTROL;
    }
}