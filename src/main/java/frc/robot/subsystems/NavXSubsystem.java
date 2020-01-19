package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.NavX.*;

import frc.robot.Robot;
import frc.robot.util.Vector3;

import com.kauailabs.navx.frc.AHRS;

public class NavXSubsystem extends SubsystemBase {

    private final AHRS board = new AHRS(SPI.Port.kMXP);

    private Vector3 initialDisplacement = new Vector3();
    private double startAngle;



    public void set(Vector3 displacement, double startAngle) {
        board.reset();
        board.setAngleAdjustment(startAngle);
        this.initialDisplacement = displacement;
        this.startAngle = startAngle;
    }

    public Vector3 getDisplacement() {
        return
        new Vector3(
            board.getDisplacementX(),
            board.getDisplacementY(),
            board.getDisplacementZ()
        )
        .applyFunction(Units::metersToInches)
        .rotateZ(-startAngle)
        .add(initialDisplacement);
    }

    public Vector3 getDisplacement(FieldObject fieldObject) {
        return
        fieldObject.getPosition()
        .subtract(getDisplacement());
    }

    public double getDirection() {
        return board.getAngle() % 360;
    }

    public Vector3 getVelocity() {
        return
        new Vector3(
            board.getVelocityX(),
            board.getVelocityY(),
            board.getVelocityZ()
        )
        .applyFunction(Units::metersToInches);
    }



    public enum FieldObject {
        POWER_PORT(POWER_PORT_POSITION);

        private final Vector3 position;
        private FieldObject(Vector3 position) {
            this.position = Robot.getAlliance() == Alliance.Red ? position : position.rotateZ(180);
        }

        public Vector3 getPosition() {
            return position;
        }
    }

}