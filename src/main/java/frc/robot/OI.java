package frc.robot;

import static frc.robot.Constants.Shooter;

import frc.robot.commands.climb.*;
import frc.robot.commands.indexer.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.util.Controller;

public class OI {

  public Controller controller1, controller2;

  public OI() {
    controller1 = new Controller(0);
    controller2 = new Controller(1);

    controller1.btnY.whenPressed(new ToggleClimb());
    controller1.bmpL.whenHeld(new ToggleIntakeMotor());
    controller1.bmpR.whenHeld(new ToggleIntake());

    controller2.btnA.whenHeld(new SetShooterSpeed(Shooter.MANUAL_RPM_1));
    controller2.btnB.whenHeld(new SetShooterSpeed(Shooter.MANUAL_RPM_2));
    controller2.btnX.whenHeld(new SetShooterSpeed(Shooter.MANUAL_RPM_3));
    controller2.btnY.whenHeld(new AutoAim());
    controller2.bmpL.whenHeld(new ReverseTower());
    controller2.bmpR.whenHeld(new ToggleIndexerTower());
  }
}
