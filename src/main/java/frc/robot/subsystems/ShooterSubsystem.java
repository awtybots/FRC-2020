package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;

import static frc.robot.Constants.Shooter.*;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonSRX motor = new WPI_TalonSRX(MotorIDs.SHOOTER);

	private static double PERIOD;

    private double goalVelocity;
    private double goalAngle;
    
    public ShooterSubsystem() {
        PERIOD = Robot.getTimePeriod();

        motor.configFactoryDefault(); // reset settings
        motor.setNeutralMode(BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
        motor.configSelectedFeedbackSensor(MOTOR_FEEDBACK_DEVICE); // sets which encoder the motor is using
            
        setGoalVelocity(0);
    }

    @Override
    public void periodic() {
        doBangBang();
    }

    private void doBangBang() {
        double currentVelocity = getRevsPerSecond();
        if(currentVelocity < goalVelocity - GOAL_VELOCITY_THRESHOLD) {
            motor.set(1);
        } else if(currentVelocity > goalVelocity + GOAL_VELOCITY_THRESHOLD){
            motor.set(0);
        }
    }
    public void doFeedForward() {
		double currentVelocity = getRevsPerSecond();
        double goalAcceleration = clamp(goalVelocity - currentVelocity, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD);
        double direction = currentVelocity == 0 ? goalVelocity : Math.signum(currentVelocity);
        double voltage = (FF_S * direction) + (FF_V * currentVelocity) + (FF_A * goalAcceleration);
        motor.setVoltage(voltage);
    }



    public double getRevsPerSecond() {
        return motor.getSelectedSensorVelocity() * 10.0 / ENCODER_UNITS;
    }
    public double getAngle() {
        return 0;
    }



    public void setGoalVelocity(double goalVelocity) {
        this.goalVelocity = goalVelocity;
    }
    public void setGoalAngle(double goalAngle) {
        this.goalAngle = goalAngle;
    }



    public boolean velocityAtGoal() {
        return Math.abs(getRevsPerSecond() - goalVelocity) <= GOAL_VELOCITY_THRESHOLD;
    }
    public boolean angleAtGoal() {
        return Math.abs(getAngle() - goalAngle) <= GOAL_ANGLE_THRESHOLD;
    }
    public boolean readyToShoot() {
        return velocityAtGoal() && angleAtGoal();
    }

}