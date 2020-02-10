package frc.robot.commands.main;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drive.DriveInches;
import frc.robot.commands.drive.RotateDegrees;
import frc.robot.commands.intake.ToggleIndexerTower;
import frc.robot.commands.intake.ToggleIntake;
import frc.robot.commands.music.PlaySong;
import frc.robot.commands.music.PlaySong.Song;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.shooter.ResetNavX;
import frc.robot.util.Vector3;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

public class Auton extends ParallelCommandGroup {

    public Auton(AutonType autonType) {
        addCommands(
            getAutonSequence(autonType)
        );
    }

    private CommandBase getAutonSequence(AutonType autonType) {
        switch(autonType) {
            case SQUARE:
                return sequence(
                    start(0, 0, 0),
                    new DriveInches(24),
                    new RotateDegrees(90),
                    new DriveInches(24),
                    new RotateDegrees(90),
                    new DriveInches(24),
                    new RotateDegrees(90),
                    new DriveInches(24),
                    new RotateDegrees(90)
                );
            case SHOOT:
                return parallel(
                    start(0, 0, 0),
                    new ToggleIntake(true),
                    new ToggleIndexerTower(true),
                    new AutoShoot()
                );
            case PLAY_MUSIC:
                return new PlaySong(Song.random());
            default:
                return start(0, 0, 0);
        }
    }

    private ResetNavX start(double x, double y, double rotation) {
        return new ResetNavX(new Vector3(x, y, 0), rotation);
    }

    @SuppressWarnings("unused")
    private Trajectory getTrajectory(String name) {
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("paths/"+name+".wpilib.json");
            return TrajectoryUtil.fromPathweaverJson(path);
        } catch(IOException e) {
            DriverStation.reportError("Unable to open trajectory '"+name+"'", e.getStackTrace()); // TODO alex log
            return new Trajectory(new ArrayList<Trajectory.State>(0)); // don't return null, that'll cause errors
        }
    }

    public enum AutonType {
        DO_NOTHING,
        SHOOT,
        PLAY_MUSIC,
        SQUARE;
    }
}