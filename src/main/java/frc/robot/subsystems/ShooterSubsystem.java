package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;

import static frc.robot.Constants.Shooter.*;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonSRX flywheel = new WPI_TalonSRX(MotorIDs.SHOOTER_FLYWHEEL);
    private final WPI_TalonSRX turret = new WPI_TalonSRX(MotorIDs.SHOOTER_TURRET);

    private double PERIOD;

    private double goalVelocity = 0;
    private double currentVelocity;

    private double goalAngle = TURRET_START_ANGLE;
    private double angleFactor = 4096.0/360.0 * TURRET_RATIO; // TODO switch to 2048 for Falcon500 encoders

    private boolean readyToShoot;

    public ShooterSubsystem() {
        PERIOD = Robot.getLoopTime();

        flywheel.configFactoryDefault();
        flywheel.setNeutralMode(FLYWHEEL_BRAKE_MODE);
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        turret.configFactoryDefault();
        turret.setNeutralMode(TURRET_BRAKE_MODE);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turret.setSelectedSensorPosition((int)(TURRET_START_ANGLE * angleFactor));
    }

    @Override
    public void periodic() {
        // get encoder measurements
        boolean velocityAtGoal = Math.abs(getFlywheelRevsPerSecond() - goalVelocity) <= GOAL_VELOCITY_THRESHOLD;;
        boolean turretAtGoal = Math.abs(getTurretAngle() - goalAngle) <= TURRET_ANGLE_THRESHOLD;
        readyToShoot = velocityAtGoal && turretAtGoal;

        // shooter motor
        switch(MOTOR_CONTROL_MODE) {
            case BANGBANG:
                flywheelBangBang();
                break;
            case FEEDFORWARD:
                flywheelFeedForward();
                break;
            default:
                break;
        }

        // turret motor
        SmartDashboard.putNumber("Turret detected angle", getTurretAngle()); // TODO temp
        if(!turretAtGoal) {
            double angleOffset = goalAngle - getTurretAngle();
            SmartDashboard.putNumber("Turret angle offset", angleOffset);
            double turnSpeed = MathUtil.clamp(angleOffset, -TURRET_ANGLE_SLOW_THRESHOLD, TURRET_ANGLE_SLOW_THRESHOLD)/TURRET_ANGLE_SLOW_THRESHOLD;
            turnSpeed *= TURRET_MAX_SPEED;
            if(Math.abs(turnSpeed) < TURRET_MIN_SPEED) turnSpeed = TURRET_MIN_SPEED * Math.signum(turnSpeed);
        }

        // SmartDashboard
        SmartDashboard.putBoolean("Shooter velocity at goal", velocityAtGoal);
        SmartDashboard.putBoolean("Shooter angle at goal", turretAtGoal);
    }

    private void flywheelBangBang() {
        if(currentVelocity < goalVelocity - GOAL_VELOCITY_THRESHOLD) {
            flywheel.set(SHOOTER_BANG_BANG_SPEED);
        } else if(currentVelocity > goalVelocity + GOAL_VELOCITY_THRESHOLD){
            flywheel.set(0);
        }
    }

    private void flywheelFeedForward() {
        double constrainedGoalVelocity = clamp(goalVelocity, -MAX_REVS_PER_SECOND, MAX_REVS_PER_SECOND);
        double constrainedGoalAcceleration = clamp(goalVelocity - currentVelocity, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD);
        constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

        double voltage = (FF_S * Math.signum(constrainedGoalVelocity)) + (FF_V * constrainedGoalVelocity) + (FF_A * constrainedGoalAcceleration);
        flywheel.setVoltage(voltage);
    }

    public void setGoalFlywheelRevsPerSecond(double goalVelocity) {
        this.goalVelocity = goalVelocity;
    }
    private double getFlywheelRevsPerSecond() {
        return flywheel.getSelectedSensorVelocity() / 409.6 * FLYWHEEL_RATIO;
    }

    public void setGoalTurretAngle(double angleOffset) {
        goalAngle = Math.floorMod((int)(getTurretAngle() + angleOffset), 360);
    }
    private double getTurretAngle() {
        return ((double)turret.getSelectedSensorPosition()) / angleFactor;
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }

    public enum MotorControlMode {
        BANGBANG,
        FEEDFORWARD,
        OFF;
    }

}
