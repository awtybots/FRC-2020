package frc.robot.commands.controlpanel;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleControlPanelSpinner extends CommandBase {
  public ToggleControlPanelSpinner() {
    addRequirements(controlPanelSubsystem);
  }

  @Override
  public void initialize() {
    controlPanelSubsystem.toggle(true);
  }

  @Override
  public void end(boolean interrupted) {
    controlPanelSubsystem.toggle(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
