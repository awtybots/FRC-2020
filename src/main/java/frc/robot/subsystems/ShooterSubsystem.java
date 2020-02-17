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

    private double PERIOD;

    private double goalVelocity = 0;
    private double currentVelocity = 0;
    private double currentAngle = 0;

    private double integralError = 0;
    private double lastVelocityError = 0;

    private double goalAngle = TURRET_START_ANGLE;
    private double angleFactor = 4096.0/360.0 * TURRET_RATIO;

    private boolean readyToShoot = false;

    private MotorControlMode MODE;

    public ShooterSubsystem() {
        PERIOD = Robot.getLoopTime();
        MODE = MOTOR_CONTROL_MODE;

        flywheel.configFactoryDefault();
        flywheel.setNeutralMode(FLYWHEEL_BRAKE_MODE);
        flywheel.setInverted(true);
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        flywheel.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms);
        flywheel.configVelocityMeasurementWindow(8);

        turret.configFactoryDefault();
        turret.setNeutralMode(TURRET_BRAKE_MODE);
        turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turret.setSelectedSensorPosition((int)(TURRET_START_ANGLE * angleFactor));

        SmartDashboard.setDefaultNumber("PID_P", PID_P); // TODO temp
        SmartDashboard.setDefaultNumber("PID_I", PID_I);
        SmartDashboard.setDefaultNumber("PID_D", PID_D);
    }

    @Override
    public void periodic() {
        // get encoder measurements
        currentVelocity = flywheel.getSelectedSensorVelocity() * 10.0 / 2048.0 * FLYWHEEL_RATIO; // TODO testing
        currentAngle = ((double)turret.getSelectedSensorPosition()) / angleFactor;
        boolean velocityAtGoal = Math.abs(currentVelocity - goalVelocity) <= FLYWHEEL_GOAL_VELOCITY_THRESHOLD;
        boolean turretAtGoal = Math.abs(currentAngle - goalAngle) <= TURRET_GOAL_ANGLE_THRESHOLD;
        readyToShoot = velocityAtGoal && turretAtGoal;

        // shooter motor
        MODE.getFunction(this).run();

        // turret motor
        SmartDashboard.putNumber("Turret detected angle", currentAngle); // TODO temp
        if(!turretAtGoal) {
            double angleOffset = goalAngle - currentAngle;
            SmartDashboard.putNumber("Turret angle offset", angleOffset);
            double turnSpeed = MathUtil.clamp(angleOffset/TURRET_ANGLE_SLOW_THRESHOLD, -1.0, 1.0) * TURRET_MAX_SPEED;
            if(Math.abs(turnSpeed) < TURRET_MIN_SPEED) turnSpeed = TURRET_MIN_SPEED * Math.signum(turnSpeed);
        }

        // SmartDashboard
        SmartDashboard.putNumber("Shooter RPM", currentVelocity*60.0);
        SmartDashboard.putNumber("Shooter goal RPM", goalVelocity*60.0);
        SmartDashboard.putBoolean("Shooter velocity at goal", velocityAtGoal);
        SmartDashboard.putBoolean("Shooter angle at goal", turretAtGoal);
    }

    private void flywheelFeedforward() {
        double goalAcceleration = goalVelocity - currentVelocity;
        double voltage
            = (FF_S * Math.signum(goalVelocity))
            + (FF_V * goalVelocity)
            + (FF_A * goalAcceleration);
        flywheel.setVoltage(voltage);
    }

    private void flywheelPID() {
        double velocityError = goalVelocity - currentVelocity;
        double accelerationError = (velocityError - lastVelocityError) / PERIOD;
        lastVelocityError = velocityError;
        integralError = clamp(integralError + (velocityError * PERIOD), -INTEGRAL_MAX/PID_I, INTEGRAL_MAX/PID_I);
        double percentOutput
            = (SmartDashboard.getNumber("PID_P", PID_P) * velocityError)
            + (SmartDashboard.getNumber("PID_I", PID_I) * integralError)
            + (SmartDashboard.getNumber("PID_D", PID_D) * accelerationError);
        // double percentOutput
        //     = (PID_P * velocityError)
        //     + (PID_I * integralError)
        //     + (PID_D * accelerationError); // TODO change back to this
        percentOutput = clamp(percentOutput, FLYWHEEL_MIN_OUTPUT, FLYWHEEL_MAX_OUTPUT) * Math.signum(goalVelocity);
        SmartDashboard.putNumber("Shooter percent output", percentOutput);
        flywheel.set(percentOutput);
    }

    public void setFlywheelGoalVelocity(double goalVelocity) {
        integralError = 0;
        this.goalVelocity = clamp(goalVelocity, -FLYWHEEL_MAX_VELOCITY, FLYWHEEL_MAX_VELOCITY);
    }

    public double getTurretAngle() {
        return currentAngle;
    }
    public void setTurretGoalAngle(double angleOffset) {
        goalAngle = Math.floorMod((int)(currentAngle + angleOffset), 360);
    }

    public boolean readyToShoot() {
        return readyToShoot;
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
