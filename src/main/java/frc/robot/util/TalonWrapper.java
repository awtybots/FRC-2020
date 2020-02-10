package frc.robot.util;

import java.util.function.Function;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

public class TalonWrapper implements SpeedController {

    public enum MotorType {
	    TALON_SRX(TalonWrapper::getTalonSRX, 4096.0, FeedbackDevice.CTRE_MagEncoder_Relative),
	    TALON_FX(TalonWrapper::getTalonFX, 2048.0, FeedbackDevice.IntegratedSensor);

	    private Function<Integer, TalonWrapper> f;
        private double u;
        private FeedbackDevice d;
	    private MotorType(Function<Integer, TalonWrapper> f, double u, FeedbackDevice d) {
	        this.f = f;
            this.u = u;
            this.d = d;
	    }
		public Function<Integer, TalonWrapper> getMotorCreateFunction() {
	        return f;
	    }
	    public double getEncoderUnits() {
			return u;
        }
        public FeedbackDevice getFeedbackDevice() {
            return d;
        }
    }

    private static Orchestra orchestra = new Orchestra();

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
        WPI_TalonFX talonFX = new WPI_TalonFX(id);
        orchestra.addInstrument(talonFX);
        return new TalonWrapper(talonFX);
    }
    public static Orchestra getOrchestra() {
        return orchestra;
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

	public void configVelocityMeasurementPeriod(VelocityMeasPeriod period) {
        talon.configVelocityMeasurementPeriod(period);
	}

	public void configVelocityMeasurementWindow(int i) {
        talon.configVelocityMeasurementWindow(i);
	}

}