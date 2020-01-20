package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Robot.*;

public class ToggleIntake extends CommandBase {

    public ToggleIntake() {
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggle(true);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.toggle(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}