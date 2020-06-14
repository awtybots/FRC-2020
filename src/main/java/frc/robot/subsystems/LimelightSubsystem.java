package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Limelight;

import javax.annotation.CheckForNull;

import frc.robot.util.Vector3;

public class LimelightSubsystem extends SubsystemBase {

    private final NetworkTable table;

    private boolean startupDone = false;
    private Timer timer = new Timer();

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        timer.reset();
    }

    @Override
    public void periodic() {
        if(!startupDone && timer.get() > Limelight.BOOT_TIME) {
            startupDone = true;
            toggleLight(false);
        }

        Vector3 targetData = getTargetData();
        boolean targetVisible = (targetData != null);
        SmartDashboard.putBoolean("Limelight Target Visible", targetVisible);
        if(targetVisible) {
            SmartDashboard.putNumber("Limelight Angle", targetData.x);
        }
    }


    private double getDouble(String name) {
        return table.getEntry(name).getDouble(0);
    }
    private boolean setNumber(String name, Number defaultNumber) {
        return table.getEntry(name).setNumber(defaultNumber);
    }


    @CheckForNull
    public Vector3 getTargetData() {
        boolean targetExists = getDouble("tv") == 1.0;
        if(!targetExists)
            return null;

        double targetX = getDouble("tx");
        double targetY = getDouble("ty");

        return new Vector3(
            targetX - Limelight.HORIZONTAL_SKEW,
            Limelight.MOUNTING_ANGLE + targetY,
            0
        ).print("Limelight data");
    }

	public boolean toggleLight(boolean on) {
        if (! setNumber("ledMode", on ? 3 : 1) )
        {
            System.err.println("Unable to toggle light");
            return false;
        } else return true;
	}


    public boolean setPipeline(Pipeline pipeline) {
        if (! setNumber("pipeline", pipeline.num) )
        {
            System.err.println("Unable to set pipeline");
            return false;
        } else return true;
    }

    public enum Pipeline {
        POWER_PORT(0),
        LOADING_STATION(1);

        public int num;

        private Pipeline(int num) {
            this.num = num;
        }
    }

}