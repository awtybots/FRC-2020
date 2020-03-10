package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class ToggleClimb extends InstantCommand {

    @Override
    public void initialize() {
        climbSubsystem.toggleClimb(!climbSubsystem.pistonsUp);
    }

}