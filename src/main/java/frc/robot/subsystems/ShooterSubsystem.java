package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

import frc.robot.Constants.Shooter;
import static frc.robot.Robot.*;

import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX flywheel = new WPI_TalonFX(MotorIDs.FLYWHEEL);
    // PID Constants
    public final static double PID_P = 0.02;
    public final static double PID_I = 0.04;
    public final static double PID_D = 0;
    public final static double INTEGRAL_MAX = 1.0;
    // Velocity Constants
    public final static double RPS_MAX = 7000.0 / 60.0;
    public final static double RPS_THRESHOLD = 50.0 / 60.0; // RPS threshold flywheel must be within to shoot balls
    public final static double PCT_MAX = 0.9;
    public final static double PCT_MIN = 0;
    // Flywheel Constants
    public final static double FLYWHEEL_RATIO = 12.0 / 36.0 * 72.0 / 22.0; // (12:36) -> (72:22)

    private double goalVelocity = 0;
    private double currentVelocity = 0;
    private double integralError = 0;
    private double lastVelocityError = 0;

    public ShooterSubsystem() {
        flywheel.configFactoryDefault();
        flywheel.setInverted( true );
        flywheel.setNeutralMode( NeutralMode.Coast );
        flywheel.configSelectedFeedbackSensor( FeedbackDevice.IntegratedSensor );
        flywheel.configVelocityMeasurementPeriod( VelocityMeasPeriod.Period_20Ms );
        flywheel.configVelocityMeasurementWindow( 8 );

        if(Shooter.IS_TUNING) {
            SmartDashboard.setDefaultNumber("Shooter PID_P", PID_P);
            SmartDashboard.setDefaultNumber("Shooter PID_I", PID_I);
            SmartDashboard.setDefaultNumber("Shooter PID_D", PID_D);
        }
    }

    @Override
    public void periodic() {
        currentVelocity = getFlywheelVelocity();
        flywheelPID();

        SmartDashboard.putNumber("Shooter RPM", currentVelocity);
        SmartDashboard.putNumber("Shooter goal RPM", goalVelocity*60.0);
        SmartDashboard.putBoolean("Shooter velocity at goal", isVelocityAtGoal());
    }

    private void flywheelPID() {
        double percentOutput;
        double velocityError = goalVelocity - currentVelocity;
        double accelerationError = (velocityError - lastVelocityError) / PERIOD;
        lastVelocityError = velocityError;
        integralError = clamp(integralError + (velocityError * PERIOD), -INTEGRAL_MAX/PID_I, INTEGRAL_MAX/PID_I);

        if (Shooter.IS_TUNING)
            percentOutput = (SmartDashboard.getNumber("Shooter PID_P", PID_P) * velocityError)
                          + (SmartDashboard.getNumber("Shooter PID_I", PID_I) * integralError)
                          + (SmartDashboard.getNumber("Shooter PID_D", PID_D) * accelerationError);
        else
            percentOutput = (PID_P * velocityError) + (PID_I * integralError) + (PID_D * accelerationError);

        percentOutput = clamp(percentOutput, PCT_MIN, PCT_MAX) * Math.signum(goalVelocity);
        flywheel.set(percentOutput);
    }

    public void setFlywheelGoalVelocity(double v) {
        integralError = 0;
        lastVelocityError = 0;
        goalVelocity = clamp(v, 0, RPS_MAX);
    }

    public double getFlywheelVelocity() {
        return flywheel.getSelectedSensorVelocity()
            * 10.0
            / 2048.0
            * FLYWHEEL_RATIO;
    }

    public boolean isVelocityAtGoal() {
        return Math.abs(getFlywheelVelocity() - goalVelocity) <= RPS_THRESHOLD;
    }

}
