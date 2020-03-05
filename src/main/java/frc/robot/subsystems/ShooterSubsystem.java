package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Robot.*;

import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX flywheel = new WPI_TalonFX(MotorIDs.FLYWHEEL);

    private double goalVelocity = 0;
    private double currentVelocity = 0;

    private double integralError = 0;
    private double lastVelocityError = 0;

    public ShooterSubsystem() {
        flywheel.configFactoryDefault();
        flywheel.setNeutralMode(FLYWHEEL_BRAKE_MODE);
        flywheel.setInverted(true);
        flywheel.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        flywheel.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_20Ms);
        flywheel.configVelocityMeasurementWindow(8);

        if(TUNING_MODE) {
            SmartDashboard.setDefaultNumber("Shooter PID_P", PID_P);
            SmartDashboard.setDefaultNumber("Shooter PID_I", PID_I);
            SmartDashboard.setDefaultNumber("Shooter PID_D", PID_D);
        }
    }

    @Override
    public void periodic() {
        currentVelocity = flywheel.getSelectedSensorVelocity() * 10.0 / 2048.0 * FLYWHEEL_RATIO;
        flywheelPID();

        SmartDashboard.putNumber("Shooter RPM", currentVelocity*60.0);
        SmartDashboard.putNumber("Shooter goal RPM", goalVelocity*60.0);
        SmartDashboard.putBoolean("Shooter velocity at goal", isVelocityAtGoal());
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

    public void setFlywheelGoalVelocity(double v) {
        integralError = 0;
        lastVelocityError = 0;
        goalVelocity = clamp(v, 0, FLYWHEEL_MAX_VELOCITY);
    }

    public boolean isVelocityAtGoal() {
        return Math.abs(currentVelocity - goalVelocity) <= FLYWHEEL_GOAL_VELOCITY_THRESHOLD;
    }

}
