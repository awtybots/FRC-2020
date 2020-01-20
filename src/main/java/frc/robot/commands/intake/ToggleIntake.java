package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import static frc.robot.Robot.*;

public class ToggleIntake extends InstantCommand {

    private final boolean on;

    public ToggleIntake(boolean on) {
        addRequirements(intakeSubsystem);
        this.on = on;
    }
    public ToggleIntake() {
        addRequirements(intakeSubsystem);
        this.on = !intakeSubsystem.getOn();
    }

    @Override
    public void initialize() {
        intakeSubsystem.toggle(on);
    }
}