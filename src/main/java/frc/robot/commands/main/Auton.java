package frc.robot.commands.main;

import static frc.robot.Robot.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.indexer.ToggleIndexerTower;
import frc.robot.commands.shooter.SetShooterSpeed;

public class Auton extends ParallelCommandGroup {
  public Auton(AutonType autonType) {
    addCommands(getAutonSequence(autonType));
  }

  public enum AutonType {
    SHOOT_AND_MOVE_FORWARD,
    SHOOT_AND_MOVE_BACKWARD,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    DO_NOTHING,
  }

  private Command getAutonSequence(AutonType autonType) {
    switch (autonType) {
      case SHOOT_AND_MOVE_FORWARD:
        return shootAndMove(true);

      case SHOOT_AND_MOVE_BACKWARD:
        return shootAndMove(false);

      case MOVE_FORWARD:
        return move(true);

      case MOVE_BACKWARD:
        return move(false);

      default:
        return new InstantCommand();
    }
  }

  private Command shootAndMove(boolean forward) {
    return sequence(
        deadline(
            new WaitCommand(10),
            new SetShooterSpeed(4000.0 / 60.0, true),
            sequence(new WaitCommand(3), new ToggleIndexerTower(true))),
        new ToggleIndexerTower(false),
        move(forward));
  }

  private Command move(boolean forward) {
    double amt = forward ? 0.5 : -0.5;
    return sequence(
        new InstantCommand(() -> drivetrainSubsystem.setMotorOutput(amt, amt), drivetrainSubsystem),
        new WaitCommand(1),
        new InstantCommand(() -> drivetrainSubsystem.stop()));
  }
}
