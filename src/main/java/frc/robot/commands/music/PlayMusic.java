package frc.robot.commands.music;

import java.util.Random;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.TalonWrapper;

public class PlayMusic extends CommandBase {

    private Song song;
    private Orchestra orchestra;
    private boolean shufflePlay = false;

    public PlayMusic(Song song) {
        if(song == null) { // if no song was given, shuffle play songs forever
            this.song = Song.random();
            this.shufflePlay = true;
        } else {
            this.song = song;
        }
        this.orchestra = TalonWrapper.getOrchestra();
    }

    @Override
    public void initialize() {
        orchestra.loadMusic(song.getPath());
        orchestra.play();
    }

    @Override
    public void end(boolean interrupted) {
        if(shufflePlay) {
            new PlayMusic(null).schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return (!shufflePlay) || (!orchestra.isPlaying());
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

        private String path;
        private Song(String name) {
            this.path = "songs/"+name+".chrp";
        }
        public String getPath() {
            return path;
        }
		public static Song random() {
			return values()[new Random().nextInt(values().length)];
		}
    }
}