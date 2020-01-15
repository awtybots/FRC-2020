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
import static frc.robot.Constants.Positions.*;

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
        Vector3 targetDisplacement = (targetVector == null ? navX.getDisplacement(FieldObject.POWER_PORT) : targetVector.rotateZ(robotAngle)).setZ(SHOOTER_HEIGHT_OFFSET);
        Vector3 robotVelocity = navX.getVelocity().setZ(0);
        
        Vector3 ballVelocity = getOptimalBallVelocity(targetDisplacement).subtract(robotVelocity);
        double aimAngle = Math.toDegrees(Math.atan2(ballVelocity.y, ballVelocity.x)) - robotAngle;
        shooter.setGoalVelocity(getOptimalRevsPerSecond(ballVelocity));
        shooter.setGoalAngle(aimAngle);

        SmartDashboard.putString("Shooter Goal Velocity", ballVelocity.toString());
        SmartDashboard.putNumber("Shooter Goal Angle", aimAngle);
    }

    private Vector3 getOptimalBallVelocity(Vector3 targetDisplacement) {
        //double magnitudeXY = targetDisplacement.clone().setZ(0).getMagnitude();
        //double magnitudeZ = targetDisplacement.z;

        // http://farside.ph.utexas.edu/teaching/336k/Newtonhtml/node29.html

        return new Vector3();
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