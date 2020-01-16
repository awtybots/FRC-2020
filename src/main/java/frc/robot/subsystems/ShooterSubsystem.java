package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;

import static frc.robot.Constants.Shooter.*;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonSRX flywheel = new WPI_TalonSRX(MotorIDs.SHOOTER_FLYWHEEL);
    private final WPI_TalonSRX turret = new WPI_TalonSRX(MotorIDs.SHOOTER_TURRET);

	private double PERIOD;

    private double goalVelocity;
    private double goalAngle;

    private boolean readyToShoot;
    
    public ShooterSubsystem() {
        PERIOD = Robot.getTimePeriod();

        flywheel.configFactoryDefault();
        flywheel.setNeutralMode(FLYWHEEL_BRAKE_MODE);
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        turret.configFactoryDefault();
        turret.setNeutralMode(TURRET_BRAKE_MODE);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turret.setSelectedSensorPosition(0);
            
        setGoalVelocity(0);
    }

    @Override
    public void periodic() {
        boolean velocityAtGoal = Math.abs(getFlywheelRevsPerSecond() - goalVelocity) <= GOAL_VELOCITY_THRESHOLD;;
        boolean angleAtGoal = Math.abs(getTurretAngle() - goalAngle) <= GOAL_ANGLE_THRESHOLD;
        readyToShoot = velocityAtGoal && angleAtGoal;

        flywheelBangBang();
        spinTurret();
    }

    private void flywheelBangBang() {
        double currentVelocity = getFlywheelRevsPerSecond();
        if(currentVelocity < goalVelocity - GOAL_VELOCITY_THRESHOLD) {
            flywheel.set(1);
        } else if(currentVelocity > goalVelocity + GOAL_VELOCITY_THRESHOLD){
            flywheel.set(0);
        }
    }
    @SuppressWarnings("unused")
    private void flywheelFeedForward() {
		double currentVelocity = getFlywheelRevsPerSecond();
        double goalAcceleration = clamp(goalVelocity - currentVelocity, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD);
        double direction = currentVelocity == 0 ? goalVelocity : Math.signum(currentVelocity);
        double voltage = (FF_S * direction) + (FF_V * currentVelocity) + (FF_A * goalAcceleration);
        flywheel.setVoltage(voltage);
    }

    private void spinTurret() {
        double currentAngle = getTurretAngle();
        if(currentAngle > goalAngle + GOAL_ANGLE_THRESHOLD) {
            turret.set(-TURRET_SPEED);
        } else if(currentAngle < goalAngle - GOAL_ANGLE_THRESHOLD) {
            turret.set(TURRET_SPEED);
        } else {
            turret.set(0);
        }
    }



    public double getFlywheelRevsPerSecond() {
        return flywheel.getSelectedSensorVelocity() / 409.6;
    }
    public double getTurretAngle() {
        return turret.getSelectedSensorPosition() / 4096 * TURRET_RATIO * 360;
    }



    public void setGoalVelocity(double goalVelocity) {
        this.goalVelocity = goalVelocity;
    }
    public void setGoalAngle(double goalAngle) {
        this.goalAngle = goalAngle;
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }

}