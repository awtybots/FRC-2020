package frc.robot.commands.intake;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleIntakeMotor extends CommandBase {
  private boolean button;
  private boolean on;

  public ToggleIntakeMotor() {
    button = true;
  }

  public ToggleIntakeMotor(boolean on) {
    button = false;
    this.on = on;
  }

  @Override
  public void initialize() {
    if (button) {
      intakeSubsystem.toggleMotor(true);
    } else {
      intakeSubsystem.toggleMotor(on);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (button) {
      intakeSubsystem.toggleMotor(false);
    }
  }

  @Override
  public boolean isFinished() {
    return !button;
  }
}
