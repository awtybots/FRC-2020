package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;

import static frc.robot.Constants.Shooter.*;

import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX flywheel = new WPI_TalonFX(MotorIDs.SHOOTER_FLYWHEEL);
    private final WPI_TalonSRX turret = new WPI_TalonSRX(MotorIDs.SHOOTER_TURRET);

    private double PERIOD = 0.02;

    private double goalVelocity = 0;
    private double currentVelocity = 0;
    private double currentAngle = 0;
    private double lastTurnSpeed = 0;

    private double integralError = 0;
    private double lastVelocityError = 0;

    private double goalAngle = TURRET_START_ANGLE;
    private double angleFactor = 4096.0/360.0 * TURRET_RATIO;

    private boolean velocityAtGoal = false;
    private boolean turretAtGoal = false;

    private boolean turretManual = false;

    public ShooterSubsystem() {
        PERIOD = Robot.getLoopTime();

        flywheel.configFactoryDefault();
        flywheel.setNeutralMode(FLYWHEEL_BRAKE_MODE);
        flywheel.setInverted(true);
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        flywheel.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms);
        flywheel.configVelocityMeasurementWindow(8);

        turret.configFactoryDefault();
        turret.setInverted(true);
        turret.setSensorPhase(true);
        turret.setNeutralMode(TURRET_BRAKE_MODE);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turret.setSelectedSensorPosition((int)(TURRET_START_ANGLE * angleFactor));

        if(TUNING_MODE) {
            SmartDashboard.setDefaultNumber("Shooter PID_P", PID_P);
            SmartDashboard.setDefaultNumber("Shooter PID_I", PID_I);
            SmartDashboard.setDefaultNumber("Shooter PID_D", PID_D);

            SmartDashboard.setDefaultNumber("Shooter FF_S", FF_S);
            SmartDashboard.setDefaultNumber("Shooter FF_V", FF_V);
            SmartDashboard.setDefaultNumber("Shooter FF_A", FF_A);
        }
    }

    @Override
    public void periodic() {
        // get encoder measurements
        currentVelocity = flywheel.getSelectedSensorVelocity() * 10.0 / 2048.0 * FLYWHEEL_RATIO;
        currentAngle = ((double)turret.getSelectedSensorPosition()) / angleFactor;
        velocityAtGoal = Math.abs(currentVelocity - goalVelocity) <= FLYWHEEL_GOAL_VELOCITY_THRESHOLD;
        turretAtGoal = Math.abs(currentAngle - goalAngle) <= TURRET_GOAL_ANGLE_THRESHOLD;

        // shooter motor
        FLYWHEEL_MOTOR_CONTROL_MODE.getFunction(this).run();

        // turret motor
        spinTurret();

        // SmartDashboard
        SmartDashboard.putNumber("Shooter RPM", currentVelocity*60.0);
        SmartDashboard.putNumber("Shooter goal RPM", goalVelocity*60.0);
        SmartDashboard.putBoolean("Shooter velocity at goal", velocityAtGoal);
        SmartDashboard.putBoolean("Shooter angle at goal", turretAtGoal);
    }

    private void flywheelFeedforward() {
        double goalAcceleration = goalVelocity - currentVelocity;
        double voltage = TUNING_MODE
            ? (SmartDashboard.getNumber("Shooter FF_S", FF_S) * Math.signum(goalVelocity))
            + (SmartDashboard.getNumber("Shooter FF_V", FF_V) * goalVelocity)
            + (SmartDashboard.getNumber("Shooter FF_A", FF_A) * goalAcceleration)

            : (FF_S * Math.signum(goalVelocity))
            + (FF_V * goalVelocity)
            + (FF_A * goalAcceleration);
        flywheel.setVoltage(voltage);
    }
    private void flywheelPID() {
        double velocityError = goalVelocity - currentVelocity;
        double accelerationError = (velocityError - lastVelocityError) / PERIOD;
        lastVelocityError = velocityError;
        integralError = clamp(integralError + (velocityError * PERIOD), -INTEGRAL_MAX/PID_I, INTEGRAL_MAX/PID_I);
        double percentOutput = TUNING_MODE
            ? (SmartDashboard.getNumber("Shooter PID_P", PID_P) * velocityError)
            + (SmartDashboard.getNumber("Shooter PID_I", PID_I) * integralError)
            + (SmartDashboard.getNumber("Shooter PID_D", PID_D) * accelerationError)

            : (PID_P * velocityError)
            + (PID_I * integralError)
            + (PID_D * accelerationError);
        percentOutput = clamp(percentOutput, FLYWHEEL_MIN_OUTPUT, FLYWHEEL_MAX_OUTPUT) * Math.signum(goalVelocity);
        SmartDashboard.putNumber("Shooter percent output", percentOutput);
        flywheel.set(percentOutput);
    }

    public void setFlywheelGoalVelocity(double goalVelocity) {
        integralError = 0;
        this.goalVelocity = clamp(goalVelocity, 0, FLYWHEEL_MAX_VELOCITY);
    }



    private void spinTurret() {
        SmartDashboard.putNumber("Turret position", turret.getSelectedSensorPosition()); // TODO remove
        SmartDashboard.putNumber("Turret detected angle", currentAngle);
        if(!turretManual) {
            if(turretAtGoal) {
                setTurretSpeed(0, false);
            } else {
                double angleOffset = goalAngle - currentAngle;
                SmartDashboard.putNumber("Turret angle offset", angleOffset);
                double turnSpeed = MathUtil.clamp(angleOffset/TURRET_ANGLE_SLOW_THRESHOLD, -1.0, 1.0) * TURRET_MAX_SPEED;
                setTurretSpeed(turnSpeed, false);
            }
        }
    }
	public void setTurretSpeed(double turnSpeed, boolean manual) {
        turretManual = manual;
        double rampedTurnAccel = clamp(turnSpeed - lastTurnSpeed, -TURRET_MAX_ACCEL * PERIOD, TURRET_MAX_ACCEL * PERIOD);
        double rampedTurnSpeed = lastTurnSpeed + rampedTurnAccel;
        lastTurnSpeed = rampedTurnSpeed;

        if(Math.abs(rampedTurnSpeed) < TURRET_MIN_SPEED) rampedTurnSpeed = TURRET_MIN_SPEED * Math.signum(rampedTurnSpeed);
        turret.set(rampedTurnSpeed);
	}
    public double getTurretAngle() {
        return currentAngle;
    }
    public void setTurretGoalAngle(double angleOffset) {
        goalAngle = Math.floorMod((int)(currentAngle + angleOffset), 360);
    }

    public boolean readyToShoot() {
        return velocityAtGoal && turretAtGoal;
    }
    public boolean isVelocityAtGoal() {
        return velocityAtGoal;
    }

    public enum MotorControlMode {
        FEEDFORWARD,
        PID,
        OFF;

        public Runnable getFunction(ShooterSubsystem instance) {
            switch(this) {
                case FEEDFORWARD:
                    return instance::flywheelFeedforward;
                case PID:
                    return instance::flywheelPID;
                default:
                    return () -> {};
            }
        }
    }

}
