/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;
import frc.robot.util.Vector3;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Constants.Controller.*;
import static frc.robot.Robot.*;

import java.util.function.Function;

public class TeleopDrive extends CommandBase {

    public TeleopDrive() {
        addRequirements(driveTrainSubsystem);
    }

    private static double deadzone(double x, double d) {
        if(Math.abs(x) < d) {
            return 0;
        } else {
            return (x - d * Math.signum(x)) / (1.0 - d);
        }
    }

    private static double smooth(double x) {
        return ((0.6 * Math.pow(Math.abs(x), 10.0)) + (0.4 * Math.abs(x))) * Math.signum(x);
    }

    @Override
    public void execute() {
        Vector3 processedInput = DRIVE_CONTROLS.processInput(xboxController1);

        double left = processedInput.x;
        double right = processedInput.y;

        if(DRIVE_MODE == DriveMode.PERCENT || DRIVE_MODE == DriveMode.RAMPED_PERCENT) {
            driveTrainSubsystem.setMotorOutput(left * MAX_OUTPUT, right * MAX_OUTPUT);
        } else {
            driveTrainSubsystem.setGoalVelocity(left * MAX_VELOCITY, right * MAX_VELOCITY);
        }
    }



    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public enum DriveControls {
        ARCADE_DRIVE((xboxController) -> {
            double speed = smooth(deadzone(-xboxController.getY(Hand.kLeft), STICK_DEADZONE));
            double rotation = smooth(deadzone(xboxController.getX(Hand.kRight), STICK_DEADZONE));

            double left = speed + rotation;
            double right = speed - rotation;

            return new Vector3(left, right, 0);
        }),
        GTA_DRIVE((xboxController) -> {
            double speed = smooth(
                deadzone(xboxController.getTriggerAxis(Hand.kRight), TRIGGER_DEADZONE)
                - deadzone(xboxController.getTriggerAxis(Hand.kLeft), TRIGGER_DEADZONE)
            );
            double rotation = smooth(deadzone(xboxController.getX(Hand.kRight), STICK_DEADZONE));

            double left = speed + rotation;
            double right = speed - rotation;

            return new Vector3(left, right, 0);
        });

        private Function<XboxController, Vector3> controlFunction;
        private DriveControls(Function<XboxController, Vector3> cf) {
            this.controlFunction = cf;
        }

        public Vector3 processInput(XboxController controller) {
            return controlFunction.apply(controller);
        }

    }
}