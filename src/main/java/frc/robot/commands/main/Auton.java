package frc.robot.commands.main;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveInches;
import frc.robot.commands.drive.RotateDegrees;
import frc.robot.commands.intake.ToggleIndexerTower;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.music.PlayMusic;
import frc.robot.commands.music.PlayMusic.Song;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ResetNavX;
import frc.robot.util.Vector3;
import static frc.robot.Robot.*;
import static frc.robot.Constants.DriveTrain.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

public class Auton extends ParallelCommandGroup {

    public enum AutonType {
        DO_NOTHING,
        SHOOT_TEST,
        SHUFFLE_PLAY,

        SQUARE,
        PATH_1;
    }

    public Auton(AutonType autonType) {
        addCommands(
            getAutonSequence(autonType)
        );
    }

    private Command getAutonSequence(AutonType autonType) {
        switch(autonType) {

            case SHOOT_TEST: return
                parallel(
                    start(0, 0, 0),
                    toggleIntake(true),
                    autoShoot()
                );

            case SHUFFLE_PLAY: return
                playMusic(null);

            case SQUARE: return
                sequence(
                    start(0, 0, 0),
                    driveInches(24),
                    rotateDegrees(90),
                    driveInches(24),
                    rotateDegrees(90),
                    driveInches(24),
                    rotateDegrees(90),
                    driveInches(24),
                    rotateDegrees(90)
                );

            case PATH_1: return
                sequence(
                    autoShoot(),
                    toggleIntake(true),
                    toggleIndexerTower(true),
                    driveTrajectory("GoToCenter", true),
                    waitSeconds(3),
                    driveTrajectory("GoToControlPanel", false)
                );

            default: return
                start(0, 0, 0);

        }
    }

    private Trajectory loadTrajectory(String name) {
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/"+name+".wpilib.json");
            return TrajectoryUtil.fromPathweaverJson(path);
        } catch(IOException e) {
            DriverStation.reportError("Unable to open trajectory '"+name+"'", e.getStackTrace()); // TODO alex log
            return new Trajectory(new ArrayList<Trajectory.State>(0)); // don't return null, that'll cause errors
        }
    }
    private Command driveTrajectory(String name, boolean first) {
        Trajectory trajectory = loadTrajectory(name);
        if(first) {
            return sequence(
                new ResetNavX(
                    new Vector3(trajectory.getInitialPose()),
                    trajectory.getInitialPose().getRotation().getDegrees()
                ),
                driveTrajectory(trajectory)
            );
        } else {
            return driveTrajectory(trajectory);
        }
    }
    private Command driveTrajectory(Trajectory trajectory) {
        return new RamseteCommand(
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
    }


    private ResetNavX start(double x, double y, double rotation) {
        return new ResetNavX(new Vector3(x, y, 0), rotation);
    }
    private AutoShoot autoShoot() {
        return new AutoShoot();
    }
    private ToggleIntake toggleIntake(boolean on) {
        return new ToggleIntake(on);
    }
    private ToggleIndexerTower toggleIndexerTower(boolean on) {
        return new ToggleIndexerTower(on);
    }
    private WaitCommand waitSeconds(double t) {
        return new WaitCommand(t);
    }
    private DriveInches driveInches(double in) {
        return new DriveInches(in);
    }
    private RotateDegrees rotateDegrees(double deg) {
        return new RotateDegrees(deg);
    }
    private PlayMusic playMusic(Song song) {
        return new PlayMusic(song);
    }
}