/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static edu.wpi.first.wpiutil.math.MathUtil.clamp;
import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.subsystems.DrivetrainSubsystem.MotorGroup.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap.MotorIDs;

public class DrivetrainSubsystem extends SubsystemBase {
  // --- Motors --- //
  private static WPI_TalonFX motorL1 = new WPI_TalonFX(MotorIDs.DRIVE_L1);
  private static WPI_TalonFX motorL2 = new WPI_TalonFX(MotorIDs.DRIVE_L2);
  private static WPI_TalonFX motorR1 = new WPI_TalonFX(MotorIDs.DRIVE_R1);
  private static WPI_TalonFX motorR2 = new WPI_TalonFX(MotorIDs.DRIVE_R2);
  // --- NavX --- //
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  // --- Constants --- //
  /// ----- Safety ----- ///
  public static final double PCT_MIN = 0;
  public static final double PCT_MAX = 0.9;
  public static final double PCT_ACCELERATION_MAX = 0.6;
  public static final double VELOCITY_MAX = 36; // inches/second
  public static final double ACCELERATION_MAX = 6; // inches/second^2
  /// ----- PID ----- ///
  public static final double PID_P = 0.02;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
  public static final double MAX_INTEGRAL = 1.0;
  /// ----- Feedforward ----- ///
  public static final double FF_S = 1.3; // voltage required to move a wheel any amount
  public static final double FF_V =
      0.12; // voltage required to sustain a wheel's speed moving 1in/second
  public static final double FF_A =
      0.1; // voltage required to accelerate wheel at 1in/second^2

  public DrivetrainSubsystem() {
    for (WPI_TalonFX motor : ALL.motorList) {
      motor.configFactoryDefault();
      motor.setNeutralMode(NeutralMode.Coast);
      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

      if (DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
        motor.configOpenloopRamp(1.0 / PCT_ACCELERATION_MAX);
      }
    }

    for (WPI_TalonFX motor : RIGHT.motorList) {
      motor.setInverted(TalonFXInvertType.Clockwise);
    }

    if (IS_TUNING) {
      SmartDashboard.setDefaultNumber("DriveTrain PID_P", PID_P);
      SmartDashboard.setDefaultNumber("DriveTrain PID_I", PID_I);
      SmartDashboard.setDefaultNumber("DriveTrain PID_D", PID_D);

      SmartDashboard.setDefaultNumber("DriveTrain FF_S", FF_S);
      SmartDashboard.setDefaultNumber("DriveTrain FF_V", FF_V);
      SmartDashboard.setDefaultNumber("DriveTrain FF_A", FF_A);
    }
  }

  @Override
  public void periodic() {
    if (DRIVE_MODE == DriveMode.VELOCITY || DRIVE_MODE == DriveMode.RAMPED_VELOCITY) {
      if (MOTOR_CONTROL_MODE == MotorControlMode.PID) {
        LEFT.drivePID();
        RIGHT.drivePID();
      } else {
        LEFT.driveFeedforward();
        RIGHT.driveFeedforward();
      }
    }
  }

  // DRIVE COMMAND FUNCTIONS

  public void setMotorOutput(double left, double right) {
    LEFT.setMotorOutput(left * PCT_MAX);
    RIGHT.setMotorOutput(right * PCT_MAX);
  }

  public void setGoalVelocity(double left, double right) {
    LEFT.setGoalVelocity(left * VELOCITY_MAX);
    RIGHT.setGoalVelocity(right * VELOCITY_MAX);
  }

  public void stop() {
    setGoalVelocity(0, 0);
    setMotorOutput(0, 0);
  }

  // SENSORS

  public void resetSensors() {
    for (WPI_TalonFX motor : ALL.motorList) {
      motor.setSelectedSensorPosition(0);
    }
    navX.reset();
  }

  public double getRotation() {
    return -navX.getAngle();
  }

  // ENUMS

  public enum MotorControlMode {
    PID,
    FEEDFORWARD;
  }

  public enum DriveMode {
    PERCENT,
    RAMPED_PERCENT,
    VELOCITY,
    RAMPED_VELOCITY;
  }

  public enum MotorGroup {
    LEFT(new WPI_TalonFX[] {motorL1, motorL2}),
    RIGHT(new WPI_TalonFX[] {motorR1, motorR2}),
    ALL(new WPI_TalonFX[] {motorL1, motorL2, motorR1, motorR2});

    private WPI_TalonFX[] motorList;

    private double goalVelocity = 0;
    private double integralError = 0;
    private double lastVelocityError = 0;

    private MotorGroup(WPI_TalonFX[] motorList) {
      this.motorList = motorList;
    }

    public double getWheelDistance() {
      return motorList[0].getSelectedSensorPosition()
          / 2048.0 // --> revs
          * WHEEL_CIRCUMFERENCE; // --> distance (inches)
    }

    public double getWheelVelocity() {
      return motorList[0].getSelectedSensorVelocity()
          / 2048.0 // --> revs/100ms
          * 10.0 // --> revs/second
          * WHEEL_CIRCUMFERENCE; // --> inches/second
    }

    private void setGoalVelocity(double goalVelocity) {
      this.integralError = 0;
      this.lastVelocityError = 0;
      this.goalVelocity = clamp(goalVelocity, -VELOCITY_MAX, VELOCITY_MAX);
    }

    private void drivePID() {
      double currentVelocity = getWheelVelocity();
      double goalAcceleration = goalVelocity - currentVelocity;
      double velocityError =
          DRIVE_MODE == DriveMode.RAMPED_VELOCITY
              ? clamp(
                  goalAcceleration,
                  -ACCELERATION_MAX * Constants.PERIOD,
                  ACCELERATION_MAX * Constants.PERIOD)
              : goalAcceleration;
      double accelerationError = (velocityError - lastVelocityError) / Constants.PERIOD;
      lastVelocityError = velocityError;
      integralError =
          clamp(
              integralError + (velocityError * Constants.PERIOD),
              -MAX_INTEGRAL / PID_I,
              MAX_INTEGRAL / PID_I);

      double P =
          (IS_TUNING ? SmartDashboard.getNumber("DriveTrain PID_P", PID_P) : PID_P) * velocityError;
      double I =
          (IS_TUNING ? SmartDashboard.getNumber("DriveTrain PID_I", PID_I) : PID_I) * integralError;
      double D =
          (IS_TUNING ? SmartDashboard.getNumber("DriveTrain PID_D", PID_D) : PID_D)
              * accelerationError;

      double output = P + I + D;
      setMotorOutput(output);
    }

    private void driveFeedforward() {
      double currentVelocity = getWheelVelocity();
      double goalAcceleration = goalVelocity - currentVelocity;
      double constrainedGoalAcceleration =
          DRIVE_MODE == DriveMode.RAMPED_VELOCITY
              ? clamp(
                  goalAcceleration,
                  -ACCELERATION_MAX * Constants.PERIOD,
                  ACCELERATION_MAX * Constants.PERIOD)
              : goalAcceleration;
      double constrainedGoalVelocity = currentVelocity + constrainedGoalAcceleration;

      double S =
          (IS_TUNING ? SmartDashboard.getNumber("DriveTrain FF_S", FF_S) : FF_S)
              * Math.signum(constrainedGoalVelocity);
      double V =
          (IS_TUNING ? SmartDashboard.getNumber("DriveTrain FF_V", FF_V) : FF_V)
              * constrainedGoalVelocity;
      double A =
          (IS_TUNING ? SmartDashboard.getNumber("DriveTrain FF_A", FF_A) : FF_A)
              * constrainedGoalAcceleration;

      double voltage = S + V + A;
      setMotorOutput(voltage / 12.0);
    }

    private void setMotorOutput(double pct) {
      if (Math.abs(pct) < PCT_MIN) {
        pct = 0;
      } else {
        pct = clamp(pct, -PCT_MAX, PCT_MAX);
      }
      for (WPI_TalonFX motor : motorList) {
        motor.set(pct);
      }
    }
  }
}
