package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveInches extends PIDCommand {

    public DriveInches(PIDController controller, DoubleSupplier measurementSource, double setpoint, DoubleConsumer useOutput, Subsystem[] requirements) {
        super(controller, measurementSource, setpoint, useOutput, requirements);
    }

}