package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlPanelSpinner;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.ColorSensorSubsystem.PanelColor;
import frc.robot.subsystems.ControlPanelSpinnerSubsystem;

public class AutoSpinControlPanel extends CommandBase {

    private final ControlPanelSpinnerSubsystem spinner;
    private final ColorSensorSubsystem sensor;

    private ControlType controlType;

    private PanelColor goalColor;

    private PanelColor lastColor;
    private int colorChanges;

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
            controlType = ControlType.ROTATION_CONTROL;
            lastColor = sensor.getCurrentColor();
            colorChanges = 0;
        }

        spinner.toggle(true);
    }

    @Override
    public void execute() {
        if(controlType == ControlType.ROTATION_CONTROL) {
            PanelColor currentColor = sensor.getCurrentColor();
            if(currentColor != lastColor) {
                colorChanges++;
                lastColor = currentColor;
                SmartDashboard.putNumber("Color changes", colorChanges);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        spinner.toggle(false);
    }

    @Override
    public boolean isFinished() {
        if(controlType == ControlType.ROTATION_CONTROL) {
            return colorChanges >= ControlPanelSpinner.COLOR_CHANGES;
        } else {
            return sensor.getCurrentColor() == goalColor;
        }
    }

    private enum ControlType {
        POSITION_CONTROL,
        ROTATION_CONTROL;
    }
}