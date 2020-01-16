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

public class AutoShoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    private final NavXSubsystem navX;

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
        Vector3 targetVector = limelight.getRelativeTargetVector();
        double robotAngle = navX.getDirection();
        Vector3 robotVelocity = navX.getVelocity().setZ(0);
        
        Vector3 targetDisplacement = targetVector == null ? navX.getDisplacement(FieldObject.POWER_PORT) : targetVector.rotateZ(robotAngle);
        targetDisplacement.setZ(FieldObject.POWER_PORT.getPosition().z - SHOOTER_HEIGHT);
        
        Vector3 ballVelocity = getOptimalBallVelocity(targetDisplacement).subtract(robotVelocity);
        double aimAngle = Math.toDegrees(Math.atan2(ballVelocity.y, ballVelocity.x)) - robotAngle;
        
        shooter.setGoalVelocity(getOptimalRevsPerSecond(ballVelocity));
        shooter.setGoalAngle(aimAngle);

        SmartDashboard.putString("Shooter Goal Velocity", ballVelocity.toString());
        SmartDashboard.putNumber("Shooter Goal Angle", aimAngle);
    }

    private Vector3 getOptimalBallVelocity(Vector3 targetDisplacement) {
        double x = targetDisplacement.clone().setZ(0).getMagnitude();
        double z = targetDisplacement.z;
        double xr = targetDisplacement.x / x;
        double yr = targetDisplacement.y / x;
        double vx = Math.cos(SHOOTER_ANGLE);
        double vz = Math.sin(SHOOTER_ANGLE);

        double v0 = Math.sqrt((GRAVITY*x*x) / ((-z + x*vz/vx)*vx*vx));

        double velocityXY = v0 * vx;
        double velocityX = velocityXY * xr;
        double velocityY = velocityXY * yr;
        double velocityZ = v0 * vz;

        return new Vector3(velocityX, velocityY, velocityZ);


        // x = cos(ang) * v0 * t
        // z = sin(ang) * v0 * t - 4.9t^2

        // t = x / (cos(ang) * v0)
        // z = sin(ang) * v0 * x / (cos(ang) * v0) - 4.9x^2/(cos(ang)^2 * v0^2)
        // sqrt(4.9x^2/(-z + sin(ang) * x / cos(ang))/cos(ang)^2) = v0


        // WITHOUT DRAG:
        //    https://youtu.be/tKrlchCio_k
        //
        // WITH DRAG:
        //    https://www.desmos.com/calculator/on4xzwtdwz
        //    https://demonstrations.wolfram.com/ProjectileWithAirDrag/
        
    }
    private double getOptimalRevsPerSecond(Vector3 ballVelocity) {
        double velocity = ballVelocity.getMagnitude();
        return velocity / WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setGoalVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}