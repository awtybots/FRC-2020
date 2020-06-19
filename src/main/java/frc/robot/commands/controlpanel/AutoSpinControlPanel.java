package frc.robot.commands.controlpanel;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    final String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if (gameData.length() > 0) {
      controlType = ControlType.POSITION_CONTROL;
      goalColor = PanelColor.fromChar(gameData.charAt(0));
    } else {
      controlType = ControlType.ROTATION_CONTROL;
      colorPasses = 0;
      startColor = controlPanelSubsystem.currentColor;
      if (startColor == PanelColor.NONE) {
        cancel();
        System.err.println("Cannot start rotation control without seeing a color!"); // log error
      }
      lastColor = startColor;
    }

    controlPanelSubsystem.toggle(true);
  }

  @Override
  public void execute() {
    if (controlType == ControlType.ROTATION_CONTROL) {
      final PanelColor currentColor = controlPanelSubsystem.currentColor;
      if (currentColor != PanelColor.NONE && lastColor != currentColor) {
        if (lastColor == startColor) {
          colorPasses++;
          SmartDashboard.putNumber("Color Passes", colorPasses);
        }
        lastColor = currentColor;
      }
    }
  }

  @Override
  public void end(final boolean interrupted) {
    controlPanelSubsystem.toggle(false);
  }

  @Override
  public boolean isFinished() {
    return (controlType == ControlType.ROTATION_CONTROL)
        ? (colorPasses >= 7)
        : (controlPanelSubsystem.currentColor == goalColor);
  }

  private enum ControlType {
    POSITION_CONTROL,
    ROTATION_CONTROL;
  }
}
