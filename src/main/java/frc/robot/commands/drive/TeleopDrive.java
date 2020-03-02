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

    public TeleopDrive() {
        addRequirements(driveTrainSubsystem);
    }

    private double deadzone(double x) {
        if(Math.abs(x) < DEADZONE) {
            return 0;
        } else {
            return x;
        }
    }

    @Override
    public void execute() {
        double speed = deadzone(-xboxController1.getY(SPEED_HAND));
        double rotation = deadzone(xboxController1.getX(ROTATION_HAND));

        if(speed < 0.0) rotation = -rotation;
        double left = speed - rotation;
        double right = speed + rotation;

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
}