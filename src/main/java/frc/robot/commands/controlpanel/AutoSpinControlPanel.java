package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlPanelSpinner;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem.PanelColor;

public class AutoSpinControlPanel extends CommandBase {

    private final ControlPanelSubsystem controlPanelSubsystem;

    private ControlType controlType;

    private PanelColor goalColor;

    private double colorPasses;
    private PanelColor startColor;
    private PanelColor lastColor;

    public AutoSpinControlPanel(ControlPanelSubsystem controlPanelSubsystem) {
        addRequirements(controlPanelSubsystem);
        this.controlPanelSubsystem = controlPanelSubsystem;
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
            lastColor = startColor;
        }

        controlPanelSubsystem.toggle(true);
    }

    @Override
    public void execute() {
        if(controlType == ControlType.ROTATION_CONTROL) {
            PanelColor currentColor = controlPanelSubsystem.getCurrentColor();
            if(lastColor != currentColor) {
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