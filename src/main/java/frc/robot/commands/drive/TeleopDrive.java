/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem.DriveMode;
import frc.robot.util.Vector3;
import frc.robot.util.Controller;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Constants.Controller.*;
import static frc.robot.Robot.*;

import java.util.function.Function;

public class TeleopDrive extends CommandBase {

    public TeleopDrive() {
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        Vector3 processedInput = DRIVE_CONTROLS.processInput(oi.firstController);

        double left = processedInput.x;
        double right = processedInput.y;

        drivetrainSubsystem.setMotorOutput(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static double smooth(double x) {
        return ((0.6 * Math.pow(Math.abs(x), 10.0)) + (0.4 * Math.abs(x))) * Math.signum(x);
    }

    public enum DriveControls { // Maps a mode of input to the appropriate method
        ARCADE_DRIVE((xboxController) -> {
            double speed = smooth( -xboxController.getY(Hand.kLeft) ); // https://www.desmos.com/calculator/uc1689lozj
            double rotation = smooth(xboxController.getX(Hand.kRight) );

            double left = speed + rotation;
            double right = speed - rotation;

            return new Vector3(left, right, 0);
        }),
        GTA_DRIVE((xboxController) -> {
            double speed = smooth(
                  xboxController.getTrigger(Hand.kRight)
                - xboxController.getTrigger(Hand.kLeft)
            );
            double rotation = smooth(xboxController.getX(Hand.kRight));

            double left = speed + rotation;
            double right = speed - rotation;

            return new Vector3(left, right, 0);
        });

        private Function<Controller, Vector3> controlFunction;

        private DriveControls(Function<Controller, Vector3> cf) {
            this.controlFunction = cf;
        }

        public Vector3 processInput(Controller controller) {
            return controlFunction.apply(controller);
        }

    }
}
