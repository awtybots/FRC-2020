package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Vector3;
import static frc.robot.Constants.Shooter.*;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    private Vector3 targetVector = new Vector3();

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        boolean targetExists = getDouble("tv") == 1.0;
        if(!targetExists) {
            setTargetVector(null);
            return;
        }

        double targetOffsetAngleHorizontal = getDouble("tx");
        double targetOffsetAngleVertical = getDouble("ty");
        double targetArea = getDouble("ta");
        double targetSkew = getDouble("ts");

        setTargetVector(new Vector3( // TODO do math
            0,
            TARGET_HEIGHT - SHOOTER_HEIGHT,
            0
        ));
    }

    private double getDouble(String name) {
        return table.getEntry(name).getDouble(0);
    }

    private void setTargetVector(Vector3 vector) {
        this.targetVector = vector;
    }
    public Vector3 getTargetVector() {
        return targetVector;
    }

}