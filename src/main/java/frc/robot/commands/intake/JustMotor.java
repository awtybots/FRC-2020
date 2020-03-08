package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;

public class JustMotor extends CommandBase {

    private boolean button;
    private boolean on;

    public JustMotor() {
        button = true;
    }
    public JustMotor(boolean on) {
        button = false;
        this.on = on;
    }

    @Override
    public void initialize() {
        if(button) {
            intakeSubsystem.justMotor(true);
        } else {
            intakeSubsystem.justMotor(on);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(button) {
            intakeSubsystem.justMotor(false);
        }
    }

    @Override
    public boolean isFinished() {
        return !button;
    }
}