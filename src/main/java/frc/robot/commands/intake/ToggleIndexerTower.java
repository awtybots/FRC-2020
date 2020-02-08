package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.*;

public class ToggleIndexerTower extends CommandBase {

    private boolean button;
    private boolean on;

    public ToggleIndexerTower() {
        addRequirements(indexerTowerSubsystem);
        button = true;
    }
    public ToggleIndexerTower(boolean on) {
        addRequirements(indexerTowerSubsystem);
        button = false;
        this.on = on;
    }


    @Override
    public void initialize() {
        if(button) {
            indexerTowerSubsystem.toggle(true);
        } else {
            indexerTowerSubsystem.toggle(on);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(button) {
            indexerTowerSubsystem.toggle(false);
        }
    }

    @Override
    public boolean isFinished() {
        return !button;
    }
}