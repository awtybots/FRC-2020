package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.MotorIDs;
import frc.robot.util.Vector3;

import static frc.robot.Constants.Shooter.*;
import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonSRX motor = new WPI_TalonSRX(MotorIDs.SHOOTER);

	private static double PERIOD;

    private double goalVelocity;
    
    public ShooterSubsystem() {
        PERIOD = Robot.getTimePeriod();

        motor.configFactoryDefault(); // reset settings
        motor.setNeutralMode(BRAKE_MODE); // sets the brake mode for all motors (called NeutralMode)
        motor.configSelectedFeedbackSensor(MOTOR_FEEDBACK_DEVICE); // sets which encoder the motor is using
            
        setGoalVelocity(0);
    }

    @Override
    public void periodic() {
        motor.setVoltage(calculateFF());
    }

    private double calculateFF() {
		double currentVelocity = getRevsPerSecond();
		double goalAcceleration = clamp(goalVelocity - currentVelocity, -MAX_ACCELERATION * PERIOD, MAX_ACCELERATION * PERIOD);
        return (FF_S * Math.signum(currentVelocity)) + (FF_V * currentVelocity) + (FF_A * goalAcceleration);
    }

    private double getRevsPerSecond() {
        return motor.getSelectedSensorVelocity() * 10.0 / ENCODER_UNITS;
    }



    public void setGoalVelocity(double goalVelocity) {
        this.goalVelocity = goalVelocity;
    }

    private Vector3 getOptimalBallVelocity(Vector3 targetDisplacement) {
        return new Vector3();
    }

    private double getOptimalRevsPerSecond(Vector3 ballVelocity) {
        return 0;
    }

    public void aimTowards(Vector3 targetDisplacement, Vector3 currentVelocity) {
        Vector3 ballVelocity = getOptimalBallVelocity(targetDisplacement).subtract(currentVelocity);
        setGoalVelocity(getOptimalRevsPerSecond(ballVelocity));
        double aimAngle = Math.atan2(ballVelocity.x, ballVelocity.z);

        SmartDashboard.putString("Shooter Velocity", ballVelocity.toString());
        SmartDashboard.putNumber("Shooter Aim Angle", aimAngle);
        // rotate turret
    }
    public boolean atGoal() {
        return Math.abs(getRevsPerSecond() - goalVelocity) <= GOAL_VELOCITY_THRESHOLD;
    }

}