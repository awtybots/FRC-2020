package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;

public class ToggleIntake extends CommandBase {

    private boolean button;
    private boolean on;

    public ToggleIntake() {
        addRequirements(intakeSubsystem);
        button = true;
    }
    public ToggleIntake(boolean on) {
        addRequirements(intakeSubsystem);
        button = false;
        this.on = on;
    }

    @Override
    public void initialize() {
        if(button) {
            intakeSubsystem.toggle(true);
        } else {
            intakeSubsystem.toggle(on);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(button) {
            intakeSubsystem.toggle(false);
        }
    }

    @Override
    public boolean isFinished() {
        return !button;
    }
}