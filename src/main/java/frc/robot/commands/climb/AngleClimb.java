package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import static frc.robot.Robot.*;

public class AngleClimb extends InstantCommand {

    @Override
    public void initialize() {
        climbSubsystem.angleClimb(!climbSubsystem.anglePistonsUp);
    }

}