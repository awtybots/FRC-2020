package frc.robot;

import frc.robot.util.Controller;
import static frc.robot.Constants.Shooter.*;

import frc.robot.commands.climb.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.indexer.*;

public class OI {

    public Controller firstController, secondController;

    public OI() {
        firstController  = new Controller( 0 );
        secondController = new Controller( 1 );

        firstController.btnY.whenPressed(new ToggleClimb());
        firstController.bmpL.whenHeld(new ToggleIntakeMotor());
        firstController.bmpR.whenHeld(new ToggleIntake());

        secondController.btnA.whenHeld(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_1));
        secondController.btnB.whenHeld(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_2));
        secondController.btnX.whenHeld(new SetShooterSpeed(FLYWHEEL_TELEOP_SPEED_3));
        secondController.btnY.whenHeld(new AutoAim());
        secondController.bmpL.whenHeld(new ReverseTower());
        secondController.bmpR.whenHeld(new ToggleIndexerTower());
    }

}

