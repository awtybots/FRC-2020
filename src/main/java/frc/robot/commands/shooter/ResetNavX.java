package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import static frc.robot.Robot.navXSubsystem;
import frc.robot.util.Vector3;

public class ResetNavX extends InstantCommand {

    private Vector3 displacement;
    private double startAngle;

    public ResetNavX() {
        this(new Vector3(), 0);
    }
    public ResetNavX(Vector3 displacement, double startAngle) {
        this.displacement = displacement;
        this.startAngle = startAngle;
    }

    @Override
    public void initialize() {
        navXSubsystem.set(displacement, startAngle);
    }

}