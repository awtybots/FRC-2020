import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

public class Teleop extends ParallelCommandGroup {

    public Teleop(XboxController xboxController, DriveTrainSubsystem driveTrainSubsystem) {
        addCommands(
            new TeleopDrive(xboxController, driveTrainSubsystem)
        );
    }

}