package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.intake.ToggleIndexerTower;
import frc.robot.commands.intake.ToggleIntake;

public class Teleop extends ParallelCommandGroup {

    public Teleop() {
        addCommands(
            new ToggleIntake(false),
            new ToggleIndexerTower(false),
            new TeleopDrive()
        );
    }

}