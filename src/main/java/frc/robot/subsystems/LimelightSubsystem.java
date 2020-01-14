package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Vector3;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    private Vector3 targetVector = new Vector3();

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        double targetOffsetAngleHorizontal = getDouble("tx");
        double targetOffsetAngleVertical = getDouble("ty");
        double targetArea = getDouble("ta");
        double targetSkew = getDouble("ts");

        // set target vector
    }

    private double getDouble(String name) {
        return table.getEntry(name).getDouble(0);
    }

    public Vector3 getTargetVector() {
        return targetVector;
    }

}