package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.shooter.AutoShoot;

public class Teleop extends ParallelCommandGroup {

    public Teleop() {
        addCommands(
            new AutoShoot(),
            new TeleopDrive()
        );
    }

}