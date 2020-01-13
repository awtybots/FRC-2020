package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ControlPanelSpinnerSubsystem;

public class ToggleControlPanelSpinner extends InstantCommand {

    private final ControlPanelSpinnerSubsystem spinner;
    private final boolean on;

    public ToggleControlPanelSpinner(ControlPanelSpinnerSubsystem spinner, boolean on) {
        addRequirements(spinner);
        this.spinner = spinner;
        this.on = on;
    }

    @Override
    public void initialize() {
        spinner.toggle(on);
    }
}