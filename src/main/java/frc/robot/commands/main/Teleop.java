package frc.robot.commands.main;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;

public class Teleop extends ParallelCommandGroup {

    public Teleop() {
        addCommands(
            //new AutoShoot(), // TODO add back
            new TeleopDrive()
        );
    }

}