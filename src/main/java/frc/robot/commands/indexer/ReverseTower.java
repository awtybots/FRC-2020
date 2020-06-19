package frc.robot.commands.indexer;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReverseTower extends CommandBase {

  public ReverseTower() {
    addRequirements(indexerTowerSubsystem);
  }

  @Override
  public void initialize() {
    indexerTowerSubsystem.reverse();
  }

  @Override
  public void end(boolean interrupted) {
    indexerTowerSubsystem.toggle(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
