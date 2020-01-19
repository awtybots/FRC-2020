package frc.robot.commands.controlpanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class ToggleControlPanelSpinner extends CommandBase {

    private final ControlPanelSubsystem spinner;

    public ToggleControlPanelSpinner(ControlPanelSubsystem spinner) {
        addRequirements(spinner);
        this.spinner = spinner;
    }

    @Override
    public void initialize() {
        spinner.toggle(true);
    }

    @Override
    public void end(boolean interrupted) {
        spinner.toggle(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}