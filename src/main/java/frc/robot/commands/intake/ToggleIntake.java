package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class ToggleIntake extends InstantCommand {

    private final IntakeSubsystem intake;
    private final boolean on;

    public ToggleIntake(IntakeSubsystem intake, boolean on) {
        addRequirements(intake);
        this.intake = intake;
        this.on = on;
    }

    @Override
    public void initialize() {
        intake.toggle(on);
    }
}