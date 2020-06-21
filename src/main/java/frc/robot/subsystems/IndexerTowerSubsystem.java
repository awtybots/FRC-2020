package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.MotorIDs;

public class IndexerTowerSubsystem extends SubsystemBase {
  private final WPI_TalonSRX indexerL = new WPI_TalonSRX(MotorIDs.INDEXER_L);
  private final WPI_TalonSRX indexerR = new WPI_TalonSRX(MotorIDs.INDEXER_R);
  private final WPI_TalonSRX tower1 = new WPI_TalonSRX(MotorIDs.TOWER_1);
  private final WPI_TalonSRX tower2 = new WPI_TalonSRX(MotorIDs.TOWER_2);

  public static final double INDEXER_L_MOTOR_SPEED = 0.8;
  public static final double INDEXER_R_MOTOR_SPEED = 0.6;
  public static final double TOWER_MOTOR_SPEED = 0.75;

  public IndexerTowerSubsystem() {
    indexerL.configFactoryDefault();
    indexerR.configFactoryDefault();
    tower1.configFactoryDefault();
    tower2.configFactoryDefault();

    indexerL.setInverted(false);
    indexerR.setInverted(true);
    tower1.setInverted(true);
    tower2.setInverted(true);

    toggle(false);
  }

  public void toggle(boolean on) {
    indexerL.set(on ? INDEXER_L_MOTOR_SPEED : 0.0);
    indexerR.set(on ? INDEXER_R_MOTOR_SPEED : 0.0);
    tower1.set(on ? TOWER_MOTOR_SPEED : 0.0);
    tower2.set(on ? TOWER_MOTOR_SPEED : 0.0);
  }

  public void reverse() {
    tower1.set(-TOWER_MOTOR_SPEED);
    tower2.set(-TOWER_MOTOR_SPEED);
  }
}
