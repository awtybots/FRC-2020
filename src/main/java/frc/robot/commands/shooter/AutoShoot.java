package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem.Pipeline;
import frc.robot.subsystems.NavXSubsystem.FieldObject;
import frc.robot.util.Vector3;
import static frc.robot.Constants.Shooter.*;
import static java.lang.Math.*;

public class AutoShoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    private final NavXSubsystem navX;

    private double revsPerSecond = 0;
    private double aimAngle = TURRET_START_ANGLE;

    public AutoShoot(ShooterSubsystem shooter, LimelightSubsystem limelight, NavXSubsystem navX) {
        addRequirements(shooter, limelight);
        this.shooter = shooter;
        this.limelight = limelight;
        this.navX = navX;
    }

    @Override
    public void initialize() {
        limelight.setPipeline(Pipeline.POWER_PORT);
    }

    @Override
    public void execute() {
        boolean canShoot = calculateTrajectory();
        
        shooter.setGoalFlywheelRevsPerSecond(revsPerSecond);
        shooter.setGoalTurretAngle(aimAngle);

        SmartDashboard.putBoolean("Shooter trajectory possible", canShoot);

        if(canShoot && shooter.readyToShoot()) {
            // actually shoot a ball
        }
    }

    private boolean calculateTrajectory() {
        Vector3 targetVector = limelight.getRelativeTargetVector();
        Vector3 robotVelocity = navX.getVelocity().setZ(0);

        double robotAngle = navX.getDirection();
        aimAngle = toDegrees(atan2(targetVector.y, targetVector.x)) - robotAngle;
        
        Vector3 targetDisplacement = targetVector == null ? navX.getDisplacement(FieldObject.POWER_PORT) : targetVector.rotateZ(robotAngle);
        targetDisplacement.setZ(FieldObject.POWER_PORT.getPosition().z - SHOOTER_HEIGHT);
        Vector3 ballVelocity = getOptimalBallVelocity(targetDisplacement).subtract(robotVelocity);

        if(ballVelocity == null) {
            revsPerSecond = 0;
            return false;
        }
        
        revsPerSecond = ballVelocity.getMagnitude() / WHEEL_CIRCUMFERENCE;

        if(revsPerSecond > MAX_REVS_PER_SECOND) {
            revsPerSecond = 0;
            return false;
        }

        SmartDashboard.putString("Shooter Goal Velocity", ballVelocity.toString());
        SmartDashboard.putNumber("Shooter Goal Angle", Math.round(aimAngle));

        return true;
    }

    private Vector3 getOptimalBallVelocity(Vector3 targetDisplacement) {
        double x = targetDisplacement.clone().setZ(0).getMagnitude();
        double z = targetDisplacement.z;
        double xr = targetDisplacement.x / x;
        double yr = targetDisplacement.y / x;

        double v0 = sqrt(
            (GRAVITY * x * x)
            /
            ((x * tan(SHOOTER_ANGLE) - z) * cos(SHOOTER_ANGLE) * cos(SHOOTER_ANGLE) * 2)
        );

        if(v0 == Double.NaN) return null;

        double velocityXY = v0 * cos(SHOOTER_ANGLE);
        double velocityX = velocityXY * xr;
        double velocityY = velocityXY * yr;
        double velocityZ = v0 * sin(SHOOTER_ANGLE);

        return new Vector3(velocityX, velocityY, velocityZ);

        // WITHOUT DRAG:
        //    https://youtu.be/tKrlchCio_k
        //
        // WITH DRAG:
        //    https://www.desmos.com/calculator/on4xzwtdwz
        //    https://demonstrations.wolfram.com/ProjectileWithAirDrag/
        
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setGoalFlywheelRevsPerSecond(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}