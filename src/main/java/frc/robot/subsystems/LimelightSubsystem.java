package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Limelight.*;

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
        if(!startupDone && timer.get() > STARTUP_TIME) {
            startupDone = true;
            toggleLight(false);
        }
    }


    private double getDouble(String name) {
        return table.getEntry(name).getDouble(0);
    }
    private void setNumber(String name, Number number) {
        table.getEntry(name).setNumber(number);
    }


    @CheckForNull
    public Vector3 getTargetData() {
        boolean targetExists = getDouble("tv") == 1.0;
        if(!targetExists) {
            return null;
        }

        double tx = getDouble("tx");
        double ty = getDouble("ty");

        return new Vector3(
            tx - CAMERA_X_OFFSET,
            CAMERA_MOUNTING_ANGLE + ty,
            0
        ).print("Limelight data");
    }

	public void toggleLight(boolean on) {
        setNumber("ledMode", on ? 3 : 1);
	}


    public void setPipeline(Pipeline pipeline) {
        setNumber("pipeline", pipeline.num);
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
