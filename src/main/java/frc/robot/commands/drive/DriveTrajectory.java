package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.shooter.ResetNavX;

import static frc.robot.Robot.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import static frc.robot.Constants.DriveTrain.*;

public class DriveTrajectory extends RamseteCommand {

    private boolean resetNavX;
    private Trajectory trajectory;

    private DriveTrajectory(Trajectory trajectory, boolean resetNavX) {
        super(
            trajectory,
            driveTrainSubsystem::getPose,
            new RamseteController(RAM_B, RAM_Z),
            new SimpleMotorFeedforward(FF_S, FF_V, FF_A),
            new DifferentialDriveKinematics(TRACK_WIDTH),
            driveTrainSubsystem::getWheelSpeeds,
            new PIDController(PID_P, PID_I, PID_D),
            new PIDController(PID_P, PID_I, PID_D),
            driveTrainSubsystem::setMotorVoltage,
            driveTrainSubsystem
        );
        this.trajectory = trajectory;
        this.resetNavX = resetNavX;
    }

    public DriveTrajectory(String name, boolean resetNavX) {
        this(loadTrajectory(name), resetNavX);
    }

    public DriveTrajectory(String name) {
        this(name, false);
    }

    @Override
    public void initialize() {
        if(resetNavX) {
            Pose2d startPose = trajectory.getInitialPose();
            new ResetNavX(
                startPose.getTranslation().getX(),
                startPose.getTranslation().getY(),
                startPose.getRotation().getDegrees()
            ).schedule();
        }
        super.initialize();
    }

    private static Trajectory loadTrajectory(String name) {
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/"+name+".wpilib.json");
            return TrajectoryUtil.fromPathweaverJson(path);
        } catch(IOException e) {
            DriverStation.reportError("Unable to open trajectory '"+name+"'", e.getStackTrace()); // log error
            return new Trajectory(new ArrayList<Trajectory.State>(0)); // don't return null, that'll cause errors
        }
    }
}