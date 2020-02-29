package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climb.Climb;
import frc.robot.commands.climb.Climb.ClimbDirection;
import frc.robot.commands.drive.DriveInches;
import frc.robot.commands.drive.DriveTrajectory;
import frc.robot.commands.drive.RotateDegrees;
import frc.robot.commands.intake.ToggleIndexerTower;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.music.PlayMusic;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ResetNavX;
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
        PATH_1,

        SQUARE_TEST,
        AUTOSHOOT_TEST,
        CLIMB_TEST,
        SHUFFLE_PLAY;
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
                    new InstantCommand(() -> driveTrainSubsystem.setMotorOutput(0.5, 0.5), driveTrainSubsystem),
                    new WaitCommand(0.3),
                    new InstantCommand(() -> driveTrainSubsystem.stop())
                );

            case PATH_1: return
                parallel(
                    new AutoShoot(),
                    new ToggleIntake(true),
                    new ToggleIndexerTower(true),

                    sequence(
                        new DriveTrajectory("GoToCenter", true),
                        new WaitCommand(3),
                        new DriveTrajectory("GoToControlPanel")
                    )
                );

            case AUTOSHOOT_TEST: return
                parallel(
                    new ResetNavX(),
                    new ToggleIntake(true),
                    new AutoShoot()
                );

            case CLIMB_TEST: return
                sequence(
                    new Climb(ClimbDirection.UP),
                    new DriveInches(12),
                    new Climb(ClimbDirection.DOWN)
                );

            case SQUARE_TEST: return
                sequence(
                    new ResetNavX(),
                    new DriveInches(24),
                    new RotateDegrees(90),
                    new DriveInches(24),
                    new RotateDegrees(90),
                    new DriveInches(24),
                    new RotateDegrees(90),
                    new DriveInches(24),
                    new RotateDegrees(90)
                );

            case SHUFFLE_PLAY: return
                new PlayMusic();

            default: return
                new ResetNavX();

        }
    }
}