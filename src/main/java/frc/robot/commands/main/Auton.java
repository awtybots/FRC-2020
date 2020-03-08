package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.indexer.ToggleIndexerTower;
import frc.robot.commands.shooter.SetShooterSpeed;

import static frc.robot.Robot.*;

public class Auton extends ParallelCommandGroup {

    public Auton(AutonType autonType) {
        addCommands(
            getAutonSequence(autonType)
        );
    }

    public enum AutonType {
        DO_NOTHING,
        MOVE_FORWARD,
        SHOOT_AND_MOVE_FORWARD,
    }

    private Command getAutonSequence(AutonType autonType) {
        switch(autonType) {

            case MOVE_FORWARD: return
                sequence(
                    new InstantCommand(() -> driveTrainSubsystem.setMotorOutput(0.5, 0.5), driveTrainSubsystem),
                    new WaitCommand(0.3),
                    new InstantCommand(() -> driveTrainSubsystem.stop())
                );

            case SHOOT_AND_MOVE_FORWARD: return
                sequence(
                    deadline(
                        new WaitCommand(10),
                        new SetShooterSpeed(4000.0/60.0, true),
                        sequence(
                            new WaitCommand(3),
                            new ToggleIndexerTower(true)
                        )
                    ),
                    new ToggleIndexerTower(false),
                    new InstantCommand(() -> driveTrainSubsystem.setMotorOutput(0.5, 0.5), driveTrainSubsystem),
                    new WaitCommand(1),
                    new InstantCommand(() -> driveTrainSubsystem.stop())
                );

            default: return
                new InstantCommand();

        }
    }
}