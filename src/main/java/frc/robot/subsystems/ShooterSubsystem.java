package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double currentVelocity;

    private boolean turretAtGoal = true;
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
            
        setGoalFlywheelRevsPerSecond(0);
    }

    @Override
    public void periodic() {
        currentVelocity = flywheel.getSelectedSensorVelocity() / 409.6 * FLYWHEEL_RATIO;
        boolean velocityAtGoal = Math.abs(currentVelocity - goalVelocity) <= GOAL_VELOCITY_THRESHOLD;;
        readyToShoot = velocityAtGoal && turretAtGoal;

        SmartDashboard.putBoolean("Shooter velocity at goal", velocityAtGoal);
        SmartDashboard.putBoolean("Shooter angle at goal", turretAtGoal);

        //flywheelBangBang();
    }

    @SuppressWarnings("unused")
    private void flywheelBangBang() {
        if(currentVelocity < goalVelocity - GOAL_VELOCITY_THRESHOLD) {
            flywheel.set(SHOOTER_BANG_BANG_SPEED);
        } else if(currentVelocity > goalVelocity + GOAL_VELOCITY_THRESHOLD){
            flywheel.set(0);
        }
    }
    @SuppressWarnings("unused")
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
    public void spinTurret(double relativeGoalAngle) {
        //turret.set(clamp(relativeGoalAngle * TURRET_SPEED * TURRET_P, -TURRET_SPEED, TURRET_SPEED));
        turretAtGoal = relativeGoalAngle == 0.0;
    }

    public boolean readyToShoot() {
        return readyToShoot;
    }

}