package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
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

    private ArrayList<Double> rpsList = new ArrayList<>(FLYWHEEL_GOAL_RPS_AVERAGE_COUNT);

    public AutoShoot() {
        addRequirements(shooterSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(Pipeline.POWER_PORT);
    }

    @Override
    public void execute() {
        // calculate trajectory (required turret angle, initial velocity, and flywheel revs per second)
        boolean accurateTrajectory = calculateTrajectory();
        SmartDashboard.putBoolean("Shooter trajectory possible", accurateTrajectory);

        // set motor speeds
        shooterSubsystem.setFlywheelGoalVelocity(FLYWHEEL_TELEOP_SPEED); // TODO change back
        switch(AIM_MODE) {
            case DRIVE:
                double turnSpeed;
                if(abs(angleOffset) > TURRET_GOAL_ANGLE_THRESHOLD) {
                    turnSpeed = MathUtil.clamp(angleOffset, -TURRET_ANGLE_SLOW_THRESHOLD, TURRET_ANGLE_SLOW_THRESHOLD)/TURRET_ANGLE_SLOW_THRESHOLD;
                    turnSpeed *= TURRET_MAX_SPEED;
                    if(abs(turnSpeed) < TURRET_MIN_SPEED) turnSpeed = TURRET_MIN_SPEED * signum(turnSpeed);
                } else {
                    turnSpeed = 0;
                }
                driveTrainSubsystem.setMotorOutput(turnSpeed, -turnSpeed);
                break;
            case TURRET:
                shooterSubsystem.setTurretGoalAngle(angleOffset);
                break;
        }

        // if ready to shoot then shoot balls
        if(accurateTrajectory && shooterSubsystem.readyToShoot()) {
            // TODO actually shoot a ball
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setFlywheelGoalVelocity(0);
        shooterSubsystem.setTurretGoalAngle(0);
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
        switch(TRAJECTORY_CALCULATION_MODE) {
            case VISION_AND_NAVX:
                // use the NavX displacement instead (less reliable) if we A) can't see the target or B) the target we're seeing belongs to the other alliance
                useNavX = visionTargetDisplacement == null || visionTargetDisplacement.clone().setZ(0).dot(navXTargetDisplacement.clone().setZ(0)) < 0;
                // calculate the NavX offset angle, it will be changed if we see a target
                angleOffset = navXTargetDisplacement.getZAngle() - robotAngle;
                targetDisplacement = useNavX
                    ? navXTargetDisplacement
                    : visionTargetDisplacement.clone().rotateZ(robotAngle);
                break;
            case VISION_ONLY:
                angleOffset = 0;
                if(visionTargetDisplacement == null) {
                    // if we don't see a target then just rotate the turret
                    angleOffset = 90;
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

        // if shot is impossible from this point, stop motor
        if(optimalBallVelocity == null) {
            optimalRevsPerSecond = 0;
            rpsList.clear();
            return false;
        }

        // subtract robot velocity from goal velocity (for moving shots)
        optimalBallVelocity.subtract(driveTrainSubsystem.getVelocity().setZ(0));
        optimalRevsPerSecond = calculateOptimalRevsPerSecond(optimalBallVelocity.getMagnitude());

        // get aim angle for new velocity
        double desiredAngleOffset = optimalBallVelocity.getZAngle() - robotAngle;
        angleOffset = abs(desiredAngleOffset) <= TURRET_GOAL_ANGLE_THRESHOLD
            ? 0.0
            : desiredAngleOffset;

        // if shooting from this point requires too much RPM, stop motor
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

    public enum AimMode {
        TURRET,
        DRIVE;
    }
    public enum TrajectoryCalculationMode {
        VISION_ONLY,
        VISION_AND_NAVX,
        PRESET_TARGET;
    }

}