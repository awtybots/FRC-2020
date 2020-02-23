/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem.DriveMode;

import static frc.robot.Constants.DriveTrain.*;
import static frc.robot.Constants.Controller.*;
import static frc.robot.Robot.*;

public class TeleopDrive extends CommandBase {

    // this command runs the entire teleop period

    public TeleopDrive() {
        addRequirements(driveTrainSubsystem);
    }

    private double smooth(double x) {
        if(Math.abs(x) < DEADZONE) return 0;
        if(JOYSTICK_SMOOTHING == JoystickSmoothing.NONE) {
            return x;
        } else {
            return Math.pow(x, 2) * Math.signum(x);
        }
    }

    @Override
    public void execute() {
        double speed = smooth(-xboxController1.getY(SPEED_HAND));
        double rotation = smooth(xboxController1.getX(ROTATION_HAND));
        double left = speed + rotation;
        double right = speed - rotation;
        if(TELEOP_DRIVE_MODE == DriveMode.DIRECT) {
            driveTrainSubsystem.setMotorOutput(left * MAX_MOTOR_OUTPUT, right * MAX_MOTOR_OUTPUT);
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

    public enum JoystickSmoothing {
        NONE, SQUARE;
    }
}