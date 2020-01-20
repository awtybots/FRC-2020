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
        this.initialDisplacement = allianceCondition(displacement);
        this.startAngle = allianceCondition(startAngle);
    }

    public Vector3 getDisplacement() {
        return
        new Vector3(
            -board.getDisplacementY(),
            board.getDisplacementX(),
            board.getDisplacementZ()
        )
        .applyFunction(Units::metersToInches)
        .rotateZ(startAngle)
        .add(initialDisplacement);
    }

    public Vector3 getDisplacement(FieldObject fieldObject) {
        return
        fieldObject.getPosition()
        .subtract(getDisplacement());
    }

    public double getDirection() {
        return Math.floorMod((int)(-board.getAngle()+startAngle), 360);
    }

    public Vector3 getVelocity() {
        return
        new Vector3(
            -board.getVelocityY(),
            board.getVelocityX(),
            board.getVelocityZ()
        )
        .applyFunction(Units::metersToInches);
    }

    private static Vector3 allianceCondition(Vector3 blue) {
        return Robot.getAlliance() == Alliance.Blue ? blue : blue.rotateZ(180).add(new Vector3(FIELD_WIDTH, 0, 0));
    }
    private static double allianceCondition(double blue) {
        return Robot.getAlliance() == Alliance.Blue ? blue : blue + 180;
    }



    public enum FieldObject {
        POWER_PORT(POWER_PORT_POSITION);

        private final Vector3 position;
        private FieldObject(Vector3 position) {
            this.position = allianceCondition(position);
        }

        public Vector3 getPosition() {
            return position;
        }
    }

}