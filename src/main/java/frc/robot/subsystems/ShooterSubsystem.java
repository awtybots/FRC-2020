package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.MotorIDs;

import frc.robot.Constants.Shooter;
import frc.robot.Constants;

import static edu.wpi.first.wpiutil.math.MathUtil.clamp;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX flywheel = new WPI_TalonFX(MotorIDs.FLYWHEEL);
    // --- Constants ---//
        /// ----- PID ----- ///
        public final static double PID_P = 0.02;
        public final static double PID_I = 0.04;
        public final static double PID_D = 0;
        public final static double INTEGRAL_MAX = 1.0;
        /// ----- Safety ----- ///
        public final static double RPM_MAX = 7000.0;
        public final static double RPM_THRESHOLD = 50.0; // will not shoot until within threshold
        public final static double PCT_MAX = 0.9;
        public final static double PCT_MIN = 0;
        /// ----- Flywheel ----- ///
        public final static double FLYWHEEL_RATIO = (12.0 / 36.0)
                                                  * (72.0 / 22.0); // (12:36) -> (72:22)

    private double goalRPS = 0;
    private double integralError = 0;
    private double currentRPS = 0;
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

    public void flywheelPID() {
        currentRPS = getFlywheelVelocity();
        double percentOutput;
        double velocityError = goalRPS - currentRPS;
        double accelerationError = (velocityError - lastVelocityError) / Constants.PERIOD;
        lastVelocityError = velocityError;
        integralError = clamp(integralError + (velocityError * Constants.PERIOD), -INTEGRAL_MAX/PID_I, INTEGRAL_MAX/PID_I);

        if (Shooter.IS_TUNING)
            percentOutput = (SmartDashboard.getNumber("Shooter PID_P", PID_P) * velocityError)
                          + (SmartDashboard.getNumber("Shooter PID_I", PID_I) * integralError)
                          + (SmartDashboard.getNumber("Shooter PID_D", PID_D) * accelerationError);
        else
            percentOutput = (PID_P * velocityError) + (PID_I * integralError) + (PID_D * accelerationError);

        percentOutput = clamp(percentOutput, PCT_MIN, PCT_MAX) * Math.signum(goalRPS);
        flywheel.set(percentOutput);
    }

    public void setFlywheelGoalVelocity(double rpm) {
        integralError = 0;
        lastVelocityError = 0;
        goalRPS = clamp(rpm/60.0, 0, RPM_MAX/60.0);
    }

    public double getFlywheelVelocity() {
        return flywheel.getSelectedSensorVelocity()
            * 10.0
            / 2048.0
            * FLYWHEEL_RATIO;
    }

    public boolean isVelocityAtGoal() {
        return Math.abs(getFlywheelVelocity() - goalRPS) <= RPM_THRESHOLD/60.0;
    }

}
