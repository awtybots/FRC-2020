package frc.robot.commands.main;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ResetNavX;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.Vector3;

@SuppressWarnings("unused")
public class Teleop extends ParallelCommandGroup {

    // this is the overlaying command group for everything that happens in the teleop period
    // see https://docs.wpilib.org/en/latest/docs/software/commandbased/command-groups.html

    public Teleop() {
        addCommands(
            //new ToggleIntake(),
            new ResetNavX(new Vector3(), 0), // TODO test
            new TeleopDrive()
        );
    }

}