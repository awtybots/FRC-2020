package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MusicSubsystem extends SubsystemBase {

    private Orchestra orchestra;

    public MusicSubsystem() {
        orchestra = new Orchestra();
    }

    public void addInstrument(TalonFX talon) {
        orchestra.addInstrument(talon);
    }

	public void playSong(String path) {
        orchestra.loadMusic(path);
        orchestra.play();
	}

}