package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.DriveInches;
import frc.robot.commands.drive.RotateDegrees;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Auton extends ParallelCommandGroup {

     // this is the overlaying command group for everything that happens in auton
     // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Auton(DriveTrainSubsystem driveTrainSubsystem, AutonType autonType) {
        addCommands(
            // list all commands to do when auton starts
            // turn on intake
            // etc...
            getAutonSequence(driveTrainSubsystem, autonType)
        );
    }

    private CommandBase getAutonSequence(DriveTrainSubsystem driveTrainSubsystem, AutonType autonType) {
        switch(autonType) {
            case SQUARE:
                return sequence(
                    new DriveInches(driveTrainSubsystem, 24),
                    new RotateDegrees(driveTrainSubsystem, 90),
                    new DriveInches(driveTrainSubsystem, 24),
                    new RotateDegrees(driveTrainSubsystem, 90),
                    new DriveInches(driveTrainSubsystem, 24),
                    new RotateDegrees(driveTrainSubsystem, 90),
                    new DriveInches(driveTrainSubsystem, 24),
                    new RotateDegrees(driveTrainSubsystem, 90)
                );
            default:
                return new InstantCommand();
        }
    }

    public enum AutonType {
        DO_NOTHING,
        SQUARE
    }
}