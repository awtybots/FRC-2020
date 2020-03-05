package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorIDs;
import static frc.robot.Constants.Climb.*;

public class ClimbSubsystem extends SubsystemBase {

    private WPI_TalonSRX climb1 = new WPI_TalonSRX(MotorIDs.CLIMB_1);
    private WPI_TalonSRX climb2 = new WPI_TalonSRX(MotorIDs.CLIMB_2);

    public ClimbSubsystem() {
        climb1.configFactoryDefault();
        climb2.configFactoryDefault();

        climb2.setInverted(true);
    }

    public void climb(ClimbDirection dir) {
        climb1.set(dir.pct);
        climb2.set(dir.pct);
    }

    public enum ClimbDirection {
        UP(MOTOR_SPEED), DOWN(-MOTOR_SPEED), NONE(0.0);

        public double pct;
        private ClimbDirection(double pct) {
            this.pct = pct;
        }
    }

}
