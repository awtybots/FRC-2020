package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem.ClimbDirection;

import static frc.robot.Robot.*;

public class ToggleClimb extends CommandBase {

    private ClimbDirection dir;

    public ToggleClimb(ClimbDirection dir) {
        this.dir = dir;
    }

    @Override
    public void initialize() {
        climbSubsystem.climb(dir);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.climb(ClimbDirection.NONE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}