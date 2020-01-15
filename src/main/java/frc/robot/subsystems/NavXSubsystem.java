package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Vector3;

import com.kauailabs.navx.frc.AHRS;

public class NavXSubsystem extends SubsystemBase {

    private final AHRS board = new AHRS(SPI.Port.kMXP);

    public Vector3 getVelocity(boolean flat) {
        return new Vector3(
            board.getVelocityX(),
            board.getVelocityY(),
            flat ? 0 : board.getVelocityZ()
        );
    }

}