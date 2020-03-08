package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class RaiseClimb extends InstantCommand {

    @Override
    public void initialize() {
        climbSubsystem.climb(!climbSubsystem.pistonsUp);
    }

}