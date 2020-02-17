package frc.robot.commands.music;

import java.util.Random;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Robot.*;

public class PlayMusic extends CommandBase {

    private Song song;
    private boolean shufflePlay = false;

    public PlayMusic() {
        this(null);
    }
    public PlayMusic(Song song) {
        addRequirements(musicSubsystem);
        if(song == null) { // if no song was given, shuffle play songs forever
            this.song = Song.random();
            this.shufflePlay = true;
        } else {
            this.song = song;
        }
    }

    @Override
    public void initialize() {
        musicSubsystem.playSong(song.getPath());
        SmartDashboard.putString("Currently playing", song.getName());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Currently playing", "");
        if(shufflePlay) {
            new PlayMusic(null).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public enum Song { // TODO give songs meaningful names
        SONG_1("song1"),
        SONG_2("song2"),
        SONG_3("song3"),
        SONG_4("song4"),
        SONG_5("song5"),
        SONG_6("song6"),
        SONG_7("song7"),
        SONG_8("song8"),
        SONG_9("song9"),
        SONG_10("song10"),
        SONG_11("song11");

        private String name;
        private Song(String name) {
            this.name = name;
        }
        public String getPath() {
            return "songs/"+name+".chrp";
        }
        public String getName() {
            return name;
        }
		public static Song random() {
			return values()[new Random().nextInt(values().length)];
		}
    }
}