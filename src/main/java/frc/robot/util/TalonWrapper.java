package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

public class TalonWrapper implements SpeedController {

    private SpeedController speedController;
    private BaseTalon talon;

    private TalonWrapper(SpeedController talon) {
        this.speedController = talon;
        this.talon = (BaseTalon)talon;
    }

    public static TalonWrapper getTalonSRX(int id) {
        return new TalonWrapper(new WPI_TalonSRX(id));
    }
    public static TalonWrapper getTalonFX(int id) {
        return new TalonWrapper(new WPI_TalonFX(id));
    }

    @Override
    @SuppressWarnings("all")
    public void pidWrite(double output) {
        speedController.pidWrite(output);
    }

    @Override
    public void set(double speed) {
        speedController.set(speed);
    }

    @Override
    public double get() {
        return speedController.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        speedController.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return speedController.getInverted();
    }

    @Override
    public void disable() {
        speedController.disable();
    }

    @Override
    public void stopMotor() {
        speedController.stopMotor();
    }

	public void configFactoryDefault() {
        talon.configFactoryDefault();
	}

	public void setNeutralMode(NeutralMode neutralMode) {
        talon.setNeutralMode(neutralMode);
	}

	public void configSelectedFeedbackSensor(FeedbackDevice feedbackDevice) {
        talon.configSelectedFeedbackSensor(feedbackDevice);
	}

	public void setSensorPhase(boolean phase) {
        talon.setSensorPhase(phase);
	}

	public double getSelectedSensorPosition() {
		return talon.getSelectedSensorPosition();
	}

	public double getSelectedSensorVelocity() {
		return talon.getSelectedSensorVelocity();
	}

	public void setSelectedSensorPosition(int sensorPos) {
        talon.setSelectedSensorPosition(sensorPos);
	}

}