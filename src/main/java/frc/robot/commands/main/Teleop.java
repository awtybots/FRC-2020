package frc.robot.commands.main;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

@SuppressWarnings("unused")
public class Teleop extends ParallelCommandGroup {

    // this is the overlaying command group for everything that happens in the teleop period
    // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Teleop(XboxController xboxController, DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem) {
        addCommands(
            //new ToggleIntake(intakeSubsystem, true),
            new TeleopDrive(xboxController, driveTrainSubsystem)
        );
    }

}