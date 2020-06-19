package frc.robot.commands.climb;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ToggleClimb extends InstantCommand {

  @Override
  public void initialize() {
    climbSubsystem.toggleClimb(!climbSubsystem.pistonsUp);
  }
}
