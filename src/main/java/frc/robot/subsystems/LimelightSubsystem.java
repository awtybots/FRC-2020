package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Limelight.*;

import frc.robot.util.Vector3;

import javax.annotation.Nullable;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    private Pipeline currentPipeline;
    private SendableChooser<Number> ledChooser = new SendableChooser<>();

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        currentPipeline = Pipeline.POWER_PORT;
        
        ledChooser.addOption("ON", 3);
        ledChooser.setDefaultOption("OFF", 1);
        SmartDashboard.putData(ledChooser);
    }
    @Override
    public void periodic() {
        Number ledMode = ledChooser.getSelected();
        setNumber("ledMode", ledMode);
    }



    private double getDouble(String name) {
        return table.getEntry(name).getDouble(0);
    }
    private void setNumber(String name, Number number) {
        table.getEntry(name).setNumber(number);
    }



    @Nullable
    public Vector3 getRelativeTargetVector() {
        boolean targetExists = getDouble("tv") == 1.0;
        if(!targetExists) {
            return null;
        }

        double targetOffsetAngleHorizontal = getDouble("tx");
        double targetOffsetAngleVertical = getDouble("ty");
        //double targetArea = getDouble("ta");
        //double targetSkew = getDouble("ts");

        double targetHeight = currentPipeline.getTargetHeight();

        double yOffset = targetHeight / Math.tan(CAMERA_MOUNTING_ANGLE + targetOffsetAngleVertical);
        double forwardOffset = (new Vector3(0, yOffset, targetHeight)).getMagnitude();
        double xOffset = forwardOffset * Math.tan(targetOffsetAngleHorizontal);
        return new Vector3(
            xOffset,
            yOffset,
            0
        );
    }



    public void setPipeline(Pipeline pipeline) {
        this.currentPipeline = pipeline;
        setNumber("pipeline", pipeline.getID());
    }

    public enum Pipeline {
        POWER_PORT(0, SHOOTER_VISION_HEIGHT_OFFSET),
        LOADING_STATION(1, LOADING_STATION_VISION_HEIGHT_OFFSET);

        private int num;
        private double height;

        private Pipeline(int num, double height) {
            this.num = num;
            this.height = height;
        }

        private int getID() {
            return num;
        }
        private double getTargetHeight() {
            return height;
        }
    }

}