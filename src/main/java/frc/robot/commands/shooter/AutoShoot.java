package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem.Pipeline;
import frc.robot.subsystems.DriveTrainSubsystem.FieldObject;
import frc.robot.util.Vector3;
import static frc.robot.Constants.Shooter.*;
import static java.lang.Math.*;

import java.util.ArrayList;

import static frc.robot.Robot.*;

import javax.annotation.CheckForNull;

public class AutoShoot extends CommandBase {

    private double optimalRevsPerSecond = 0;
    private double angleOffset = 0;
    private double shooterAngle = toRadians(SHOOTER_ANGLE);
    private final double shooterVelocityToRPSFactor = 2.0 / FLYWHEEL_CIRCUMFERENCE / FLYWHEEL_SLIPPING_FACTOR;

    private boolean sawTarget = false;
    private boolean hitRight = false;

    private ArrayList<Double> rpsList = new ArrayList<>(FLYWHEEL_GOAL_RPS_AVERAGE_COUNT);

    public AutoShoot() {
        addRequirements(shooterSubsystem, limelightSubsystem);
        if(AUTO_SHOOT_MODE == AutoShootMode.MOVING_SHOTS) addRequirements(indexerTowerSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(Pipeline.POWER_PORT);
        limelightSubsystem.toggleLight(true);
    }

    @Override
    public void execute() {
        switch(AUTO_SHOOT_MODE) {
            case JUST_AIM_TURRET:
                shooterSubsystem.setTurretGoalAngle(calculateAngleOffset());
                break;
            case MOVING_SHOTS:
                // calculate trajectory (required turret angle, initial velocity, and flywheel revs per second)
                boolean accurateTrajectory = calculateTrajectory();
                SmartDashboard.putBoolean("Shooter trajectory possible", accurateTrajectory);

                // set motor speeds
                shooterSubsystem.setFlywheelGoalVelocity(optimalRevsPerSecond);
                shooterSubsystem.setTurretGoalAngle(angleOffset);

                // if ready to shoot then shoot balls
                boolean enableIndexerTower = (accurateTrajectory && shooterSubsystem.readyToShoot());
                indexerTowerSubsystem.toggle(enableIndexerTower);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelGoalVelocity(0);
        shooterSubsystem.setTurretGoalAngle(0);
        limelightSubsystem.toggleLight(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    @CheckForNull
    private boolean calculateTrajectory() { // calculates necessary RPM and angle from limelight data (or NavX)
        // gather navx and vision info
        double robotAngle = driveTrainSubsystem.getRotation();
        Vector3 visionTargetInfo = limelightSubsystem.getTargetInfo();
        Vector3 visionTargetDisplacement = calculateVisionTargetOffset(visionTargetInfo);
        Vector3 navXTargetDisplacement = driveTrainSubsystem.getDisplacement(FieldObject.POWER_PORT);

        // choose displacement of goal from navx or limelight
        Vector3 targetDisplacement = null;
        boolean useNavX = false;
        angleOffset = calculateAngleOffset(robotAngle, visionTargetInfo, navXTargetDisplacement);

        switch(TRAJECTORY_CALCULATION_MODE) {
            case VISION_AND_NAVX:
                // use the NavX displacement instead (less reliable) if we A) can't see the target or B) the target we're seeing belongs to the other alliance
                useNavX = visionTargetDisplacement == null || visionTargetDisplacement.clone().setZ(0).dot(navXTargetDisplacement.clone().setZ(0)) < 0;
                targetDisplacement = useNavX
                    ? navXTargetDisplacement
                    : visionTargetDisplacement.clone().rotateZ(robotAngle);
                break;
            case VISION_ONLY:
                if(visionTargetDisplacement == null) {
                    optimalRevsPerSecond = 0;
                    rpsList.clear();
                    return false;
                }
                targetDisplacement = visionTargetDisplacement.clone().rotateZ(robotAngle);
                break;
            case PRESET_TARGET:
                targetDisplacement = PRESET_TARGET_DISPLACEMENT.clone().rotateZ(robotAngle);
                break;
        }

        // calculate optimal ball velocity from displacement
        targetDisplacement.setZ(FieldObject.POWER_PORT.getPosition().z - SHOOTER_HEIGHT);
        Vector3 optimalBallVelocity = calculateOptimalBallVelocity(targetDisplacement);

        // check if shot is impossible from this point
        if(optimalBallVelocity == null) {
            optimalRevsPerSecond = 0;
            rpsList.clear();
            return false;
        }

        // subtract robot velocity from goal velocity (for moving shots)
        optimalBallVelocity.subtract(driveTrainSubsystem.getVelocity().setZ(0));
        optimalRevsPerSecond = calculateOptimalRevsPerSecond(optimalBallVelocity.getMagnitude());

        // get better aim angle for new velocity
        angleOffset = optimalBallVelocity.getZAngle() - robotAngle;

        // check if shooting from this point requires too much RPM
        if(optimalRevsPerSecond > FLYWHEEL_MAX_VELOCITY) {
            optimalRevsPerSecond = 0;
            rpsList.clear();
            return false;
        }

        // SmartDashboard
        SmartDashboard.putString("Shooter goal velocity", optimalBallVelocity.toString());
        SmartDashboard.putNumber("Shooter angle offset", angleOffset);
        SmartDashboard.putBoolean("Shooter using NavX", useNavX);

        // don't shoot if we used the NavX for displacement, wait for the camera to see the power port
        return !useNavX;
    }

    private Vector3 calculateVisionTargetOffset(Vector3 visionTargetInfo) {
        if(visionTargetInfo == null) return null;
        double yOffset = visionTargetInfo.z / tan(toRadians(visionTargetInfo.y));
        double forwardOffset = (new Vector3(0, yOffset, visionTargetInfo.z)).getMagnitude();
        double xOffset = forwardOffset * tan(toRadians(visionTargetInfo.x));
        return new Vector3(xOffset, yOffset, 0).print("Vision offset");
    }

    private Vector3 calculateOptimalBallVelocity(Vector3 targetDisplacement) {
        if(targetDisplacement == null) return null;
        double x = targetDisplacement.clone().setZ(0).getMagnitude();
        double z = targetDisplacement.z;
        double xr = targetDisplacement.x / x;
        double yr = targetDisplacement.y / x;

        double v0 = sqrt(
            (GRAVITY * x * x)
            /
            ((x * tan(shooterAngle) - z) * cos(shooterAngle) * cos(shooterAngle) * 2)
        );

        if(Double.isNaN(v0)) return null;

        double velocityXY = v0 * cos(shooterAngle);
        double velocityX = velocityXY * xr;
        double velocityY = velocityXY * yr;
        double velocityZ = v0 * sin(shooterAngle);

        return new Vector3(velocityX, velocityY, velocityZ);

        // WITHOUT DRAG:
        //    https://youtu.be/tKrlchCio_k
        //
        // WITH DRAG:
        //    https://www.desmos.com/calculator/on4xzwtdwz
        //    https://demonstrations.wolfram.com/ProjectileWithAirDrag/

    }

    private double calculateOptimalRevsPerSecond(double velocity) {
        double rpsNow = velocity * shooterVelocityToRPSFactor;
        rpsList.add(rpsNow);
        if(rpsList.size() > FLYWHEEL_GOAL_RPS_AVERAGE_COUNT) rpsList.remove(0);
        double total = 0;
        for(double rps : rpsList) {
            total += rps;
        }
        return total/(double)rpsList.size();
    }

    private double calculateAngleOffset() {
        double robotAngle = driveTrainSubsystem.getRotation();
        Vector3 visionTargetInfo = limelightSubsystem.getTargetInfo();
        Vector3 navXTargetDisplacement = driveTrainSubsystem.getDisplacement(FieldObject.POWER_PORT);

        return calculateAngleOffset(robotAngle, visionTargetInfo, navXTargetDisplacement);
    }

    private double calculateAngleOffset(double robotAngle, Vector3 visionTargetInfo, Vector3 navXTargetDisplacement) {
        switch(TRAJECTORY_CALCULATION_MODE) {
            case VISION_ONLY:
                if(visionTargetInfo == null) {
                    if(sawTarget) {
                        hitRight = false;
                        sawTarget = false;
                    }
                    if(shooterSubsystem.getTurretAngle() + TURRET_WRAP_AROUND_THRESHOLD > 360) {
                        hitRight = true;
                    }
                    if(hitRight) {
                        return -TURRET_ANGLE_SLOW_THRESHOLD;
                    } else {
                        return TURRET_ANGLE_SLOW_THRESHOLD;
                    }
                } else {
                    sawTarget = true;
                    return visionTargetInfo.x;
                }
            case VISION_AND_NAVX:
                if(visionTargetInfo == null) {
                    return navXTargetDisplacement.getZAngle() - robotAngle;
                } else {
                    return visionTargetInfo.x;
                }
            case PRESET_TARGET:
                return PRESET_TARGET_DISPLACEMENT.getZAngle();
            default:
                return 0;
        }
    }

    public enum TrajectoryCalculationMode {
        VISION_ONLY,
        VISION_AND_NAVX,
        PRESET_TARGET;
    }

    public enum AutoShootMode {
        JUST_AIM_TURRET,
        MOVING_SHOTS;
    }

}