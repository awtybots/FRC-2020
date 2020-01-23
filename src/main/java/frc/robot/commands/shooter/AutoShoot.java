package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.LimelightSubsystem.Pipeline;
import frc.robot.subsystems.DriveTrainSubsystem.FieldObject;
import frc.robot.util.Vector3;
import static frc.robot.Constants.Shooter.*;
import static java.lang.Math.*;
import static frc.robot.Robot.*;

import javax.annotation.CheckForNull;

public class AutoShoot extends CommandBase {

    private double optimalRevsPerSecond = 0;
    private double angleOffset = 0.0;
    private double shooterAngle = toRadians(SHOOTER_ANGLE);

    public AutoShoot() {
        addRequirements(shooterSubsystem, limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.setPipeline(Pipeline.POWER_PORT);
    }

    @Override
    public void execute() {
        boolean canShoot = calculateTrajectory();

        shooterSubsystem.setGoalFlywheelRevsPerSecond(optimalRevsPerSecond);

        double turnSpeed = MathUtil.clamp(angleOffset, -TURRET_ANGLE_SLOW_THRESHOLD, TURRET_ANGLE_SLOW_THRESHOLD)/TURRET_ANGLE_SLOW_THRESHOLD;
        turnSpeed *= TURRET_MAX_SPEED;
        turnSpeed += signum(turnSpeed) * TURRET_MIN_SPEED;
        switch(AIM_MODE) {
            case DRIVE:
                driveTrainSubsystem.setMotorOutput(turnSpeed, -turnSpeed);
                break;
            case TURRET:
                shooterSubsystem.spinTurret(turnSpeed);
                break;
        }


        SmartDashboard.putBoolean("Shooter trajectory possible", canShoot);

        if(canShoot && shooterSubsystem.readyToShoot()) {
            // actually shoot a ball
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setGoalFlywheelRevsPerSecond(0.0);
        shooterSubsystem.spinTurret(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }



    @CheckForNull
    private boolean calculateTrajectory() {
        double robotAngle = driveTrainSubsystem.getRotation();
        Vector3 visionTargetInfo = limelightSubsystem.getTargetInfo();

        Vector3 visionTargetDisplacement = calculateVisionTargetOffset(visionTargetInfo);
        Vector3 navXTargetDisplacement = driveTrainSubsystem.getDisplacement(FieldObject.POWER_PORT);

        // use the NavX displacement instead (less reliable) if we A) can't see the target or B) the target we're seeing belongs to the other alliance
        boolean useNavX = visionTargetDisplacement == null;// TODO add this: || visionTargetDisplacement.clone().setZ(0).dot(navXTargetDisplacement.clone().setZ(0)) < 0;

        angleOffset = useNavX && false // TODO
            ? floorMod((int)toDegrees(atan2(navXTargetDisplacement.y, navXTargetDisplacement.x)), 360) - robotAngle
            : visionTargetInfo == null || abs(visionTargetInfo.x) <= TURRET_ANGLE_THRESHOLD // TODO remove null check
                ? 0.0
                : visionTargetInfo.x;

        Vector3 targetDisplacement = useNavX
            ? navXTargetDisplacement
            : visionTargetDisplacement.rotateZ(robotAngle); // vision displacement vector is relative to front of robot, so rotate it
        targetDisplacement.setZ(FieldObject.POWER_PORT.getPosition().z - SHOOTER_HEIGHT);

        Vector3 optimalBallVelocity = calculateOptimalBallVelocity(targetDisplacement);

        if(optimalBallVelocity == null) { // shot is impossible from this point
            optimalRevsPerSecond = 0;
            return false;
        }

        optimalBallVelocity = optimalBallVelocity.subtract(driveTrainSubsystem.getVelocity().setZ(0)); // subtract robot velocity from goal velocity (for moving shots)
        optimalRevsPerSecond = calculateOptimalRevsPerSecond(optimalBallVelocity.getMagnitude());

        if(optimalRevsPerSecond > MAX_REVS_PER_SECOND) { // shooting from this point requires too much speed
            optimalRevsPerSecond = 0;
            return false;
        }

        SmartDashboard.putString("Shooter goal velocity", optimalBallVelocity.toString());
        SmartDashboard.putNumber("Shooter angle offset", angleOffset);

        return !useNavX; // don't shoot if we used the NavX for displacement, wait for the camera to see the power port
    }

    private Vector3 calculateVisionTargetOffset(Vector3 visionTargetInfo) {
        if(visionTargetInfo == null) return null;
        double yOffset = visionTargetInfo.z / tan(toRadians(visionTargetInfo.y));
        //yOffset = 95.4 - 1.64*yOffset + 0.008*yOffset*yOffset;
        double forwardOffset = (new Vector3(0, yOffset, visionTargetInfo.z)).getMagnitude();
        double xOffset = forwardOffset * tan(toRadians(visionTargetInfo.x));
        return new Vector3(xOffset, yOffset, 0).print("Vision offset");
    }

    private Vector3 calculateOptimalBallVelocity(Vector3 targetDisplacement) {
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
        return velocity * 10; // TODO
    }

    public enum AimMode {
        TURRET,
        DRIVE;
    }

}