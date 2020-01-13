package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.DriverStation;
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

    private PanelColor startColor;
    private boolean wasOnStartColor;
    private int startColorCrossings;

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
            startColor = sensor.getCurrentColor();
            wasOnStartColor = true;
            startColorCrossings = 0;
        }

        spinner.toggle(true);
    }

    @Override
    public void execute() {
        if(controlType == ControlType.ROTATION_CONTROL) {
            boolean isOnStartColor = sensor.getCurrentColor() == startColor;
            if(wasOnStartColor != isOnStartColor) {
                wasOnStartColor = isOnStartColor;
                if(isOnStartColor) {
                    startColorCrossings++;
                }
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
            return startColorCrossings >= ControlPanelSpinner.COLOR_CROSSINGS;
        } else {
            return sensor.getCurrentColor() == goalColor;
        }
    }

    private enum ControlType {
        POSITION_CONTROL,
        ROTATION_CONTROL;
    }
}