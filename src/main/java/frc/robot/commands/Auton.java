package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Auton extends ParallelCommandGroup {

     // this is the overlaying command group for everything that happens in auton
     // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Auton(DriveTrainSubsystem driveTrainSubsystem) {
        addCommands(
            // list all commands to do when auton starts
            // turn on intake
            // etc...
            sequence(
                // list of commands to run consecutively
                // drive forward x inches
                // turn y degrees
                // shoot
                // etc...
            )
        );
    }

}