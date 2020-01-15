package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.NavX.*;
import frc.robot.util.Vector3;

import com.kauailabs.navx.frc.AHRS;

public class NavXSubsystem extends SubsystemBase {

    private final AHRS board = new AHRS(SPI.Port.kMXP);

    private Vector3 displacement = new Vector3();



    public void set(Vector3 displacement, double direction) {
        board.reset();
        board.setAngleAdjustment(direction);
        this.displacement = displacement;
    }

    public Vector3 getDisplacement() {
        return new Vector3(board.getDisplacementX(), board.getDisplacementY(), board.getDisplacementZ()).add(displacement);
    }

    public Vector3 getDisplacement(FieldObject fieldObject) {
        return fieldObject.getPosition().subtract(getDisplacement());
    }

    public double getDirection() {
        return board.getAngle();
    }

    public Vector3 getVelocity() {
        return new Vector3(
            board.getVelocityX(),
            board.getVelocityY(),
            board.getVelocityZ()
        );
    }



    public enum FieldObject {
        POWER_PORT(POWER_PORT_POSITION);

        private final Vector3 position;
        private FieldObject(Vector3 position) {
            this.position = position;
        }

        public Vector3 getPosition() {
            return position;
        }
    }

}