package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import static frc.robot.Constants.IndexerTower.*;

public class IndexerTowerSubsystem extends SubsystemBase {

    private final WPI_TalonSRX indexerL = new WPI_TalonSRX(MotorIDs.INDEXER_L);
    private final WPI_TalonSRX indexerR = new WPI_TalonSRX(MotorIDs.INDEXER_R);
    private final WPI_TalonSRX tower = new WPI_TalonSRX(MotorIDs.TOWER);

    public IndexerTowerSubsystem() {
        indexerL.setInverted(true); // TODO figure out correct inversions

        toggle(false);
    }

    public void toggle(boolean on) {
        indexerL.set(on ? INDEXER_MOTOR_SPEED : 0);
        indexerR.set(on ? INDEXER_MOTOR_SPEED : 0);
        tower.set(on ? TOWER_MOTOR_SPEED : 0);
    }
}
