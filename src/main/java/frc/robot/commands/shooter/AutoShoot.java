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

import javax.annotation.CheckForNull;

public class AutoShoot extends CommandBase {

    private final ShooterSubsystem shooter;
    private final LimelightSubsystem limelight;
    private final NavXSubsystem navX;

    private double optimalRevsPerSecond = 0;
    private double spinTurret = 0;

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
        
        shooter.setGoalFlywheelRevsPerSecond(optimalRevsPerSecond);
        shooter.spinTurret(spinTurret);

        SmartDashboard.putBoolean("Shooter trajectory possible", canShoot);

        if(canShoot && shooter.readyToShoot()) {
            // actually shoot a ball
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setGoalFlywheelRevsPerSecond(0.0);
        shooter.spinTurret(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    

    @CheckForNull
    private boolean calculateTrajectory() {
        double robotAngle = navX.getDirection();
        Vector3 visionTargetInfo = limelight.getTargetInfo();
        
        Vector3 visionTargetDisplacement = calculateVisionTargetOffset(visionTargetInfo);
        Vector3 navXTargetDisplacement = navX.getDisplacement(FieldObject.POWER_PORT);
        
        // use the NavX displacement instead (less reliable) if we A) can't see the target or B) the target we're seeing belongs to the other alliance
        boolean useNavX = visionTargetDisplacement == null || visionTargetDisplacement.clone().setZ(0).dot(navXTargetDisplacement.clone().setZ(0)) < 0;

        spinTurret = useNavX
            ? toDegrees(atan2(navXTargetDisplacement.y, navXTargetDisplacement.x) + Math.PI) - robotAngle
            : abs(visionTargetInfo.x) <= TURRET_ANGLE_THRESHOLD
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

        optimalBallVelocity = optimalBallVelocity.subtract(navX.getVelocity().setZ(0)); // subtract robot velocity from goal velocity (for moving shots)
        optimalRevsPerSecond = optimalBallVelocity.getMagnitude() / WHEEL_CIRCUMFERENCE;

        if(optimalRevsPerSecond > MAX_REVS_PER_SECOND) { // shooting from this point requires too much speed
            optimalRevsPerSecond = 0;
            return false;
        }

        SmartDashboard.putString("Shooter goal velocity", optimalBallVelocity.toString());
        SmartDashboard.putNumber("Turret spin direction", spinTurret);

        return !useNavX; // don't shoot if we used the NavX for displacement, wait for the camera to see the power port
    }

    private Vector3 calculateVisionTargetOffset(Vector3 visionTargetInfo) {
        if(visionTargetInfo == null) return null;
        double yOffset = visionTargetInfo.z / tan(visionTargetInfo.y);
        double forwardOffset = (new Vector3(0, yOffset, visionTargetInfo.z)).getMagnitude();
        double xOffset = forwardOffset * tan(visionTargetInfo.x);
        return new Vector3(xOffset, yOffset, 0);
    }

    private Vector3 calculateOptimalBallVelocity(Vector3 targetDisplacement) {
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

}