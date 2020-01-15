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

    private PanelColor lastColor;
    private int colorChanges;

    public AutoSpinControlPanel(ControlPanelSubsystem controlPanelSubsystem) {
        addRequirements(controlPanelSubsystem);
        this.controlPanelSubsystem = controlPanelSubsystem;
    }

    @Override
    public void initialize() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if(gameData.length() > 0) {
            controlType = ControlType.POSITION_CONTROL;
            goalColor = PanelColor.fromChar(gameData.charAt(0));
        } else {
            controlType = ControlType.ROTATION_CONTROL;
            lastColor = controlPanelSubsystem.getCurrentColor();
            colorChanges = 0;
        }

        controlPanelSubsystem.toggle(true);
    }

    @Override
    public void execute() {
        if(controlType == ControlType.ROTATION_CONTROL) {
            PanelColor currentColor = controlPanelSubsystem.getCurrentColor();
            if(currentColor != lastColor) {
                lastColor = currentColor;
                colorChanges++;
                SmartDashboard.putNumber("Color changes", colorChanges);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        controlPanelSubsystem.toggle(false);
    }

    @Override
    public boolean isFinished() {
        return (controlType == ControlType.ROTATION_CONTROL) ? (colorChanges >= ControlPanelSpinner.COLOR_CHANGES) : (controlPanelSubsystem.getCurrentColor() == goalColor);
    }

    private enum ControlType {
        POSITION_CONTROL,
        ROTATION_CONTROL;
    }
}