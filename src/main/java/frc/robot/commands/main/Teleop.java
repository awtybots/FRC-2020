package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.intake.ToggleIndexerTower;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.ResetNavX;
import frc.robot.util.Vector3;

public class Teleop extends ParallelCommandGroup {

    // this is the overlaying command group for everything that happens in the teleop period
    // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Teleop() {
        addCommands(
            new ToggleIntake(false),
            new ToggleIndexerTower(false),
            new ResetNavX(new Vector3(), 0), // TODO temp
            new TeleopDrive()
        );
    }

}