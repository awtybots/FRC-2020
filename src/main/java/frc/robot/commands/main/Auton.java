package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.DriveInches;
import frc.robot.commands.drive.RotateDegrees;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import static frc.robot.Robot.*;

@SuppressWarnings("unused")
public class Auton extends ParallelCommandGroup {

    // this is the overlaying command group for everything that happens in auton
    // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Auton(AutonType autonType) {
        addCommands(
            //new ToggleIntake(intakeSubsystem, true),
            getAutonSequence(autonType)
        );
    }

    private CommandBase getAutonSequence(AutonType autonType) {
        switch(autonType) {
            case SQUARE:
                return sequence(
                    driveInches(24),
                    rotateDegrees(90),
                    driveInches(24),
                    rotateDegrees(90),
                    driveInches(24),
                    rotateDegrees(90),
                    driveInches(24),
                    rotateDegrees(90)
                );
            default:
                return new InstantCommand();
        }
    }

    private DriveInches driveInches(double inches) {
        return new DriveInches(inches);
    }
    private RotateDegrees rotateDegrees(double degrees) {
        return new RotateDegrees(degrees);
    }

    public enum AutonType {
        DO_NOTHING,
        SQUARE;
    }
}