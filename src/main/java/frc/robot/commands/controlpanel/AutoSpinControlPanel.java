package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Robot.*;
import frc.robot.Constants.ControlPanelSpinner;
import frc.robot.subsystems.ControlPanelSubsystem.PanelColor;

public class AutoSpinControlPanel extends CommandBase {

    private ControlType controlType;

    private PanelColor goalColor;

    private double colorPasses;
    private PanelColor startColor;
    private PanelColor lastColor;

    public AutoSpinControlPanel() {
        addRequirements(controlPanelSubsystem);
    }

    @Override
    public void initialize() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            controlType = ControlType.POSITION_CONTROL;
            goalColor = PanelColor.fromChar(gameData.charAt(0));
        } else {
            controlType = ControlType.ROTATION_CONTROL;
            colorPasses = 0;
            startColor = controlPanelSubsystem.getCurrentColor();
            if(startColor == PanelColor.NONE) {
                cancel();
                System.err.println("[AutoSpinControlPanel.java] Cannot start rotation control without seeing a color!");
            }
            lastColor = startColor;
        }

        controlPanelSubsystem.toggle(true);
    }

    @Override
    public void execute() {
        if(controlType == ControlType.ROTATION_CONTROL) {
            PanelColor currentColor = controlPanelSubsystem.getCurrentColor();
            if(currentColor != PanelColor.NONE && lastColor != currentColor) {
                if(lastColor == startColor) {
                    colorPasses++;
                    SmartDashboard.putNumber("Color Passes", colorPasses);
                }
                lastColor = currentColor;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        controlPanelSubsystem.toggle(false);
    }

    @Override
    public boolean isFinished() {
        return (controlType == ControlType.ROTATION_CONTROL)
                ? (colorPasses >= ControlPanelSpinner.COLOR_PASSES)
                : (controlPanelSubsystem.getCurrentColor() == goalColor);
    }

    private enum ControlType {
        POSITION_CONTROL, ROTATION_CONTROL;
    }
}