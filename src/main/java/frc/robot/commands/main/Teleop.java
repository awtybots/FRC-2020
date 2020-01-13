package frc.robot.commands.main;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Teleop extends ParallelCommandGroup {

    // this is the overlaying command group for everything that happens in the teleop period
    // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Teleop(XboxController xboxController, DriveTrainSubsystem driveTrainSubsystem) {
        addCommands(
            new TeleopDrive(xboxController, driveTrainSubsystem) // starts the teleoperated driving command
            // start teleoperated intake
            // start teleoperated shooter
            // etc...
        );
    }

}