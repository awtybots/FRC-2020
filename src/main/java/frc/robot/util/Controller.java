package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {

    public JoystickButton btnA, btnX, btnY, btnB, bmpL, bmpR;
    private XboxController controller;

    public Controller( int port )
    {
        controller = new XboxController(port);

        btnA = new JoystickButton(controller, Button.kA.value);
        btnB = new JoystickButton(controller, Button.kB.value);
        btnX = new JoystickButton(controller, Button.kX.value);
        btnY = new JoystickButton(controller, Button.kY.value);
        bmpL = new JoystickButton(controller, Button.kBumperLeft.value);
        bmpR = new JoystickButton(controller, Button.kBumperRight.value);
    }

    public double getY( GenericHID.Hand hand )
    {
        return controller.getY( hand );
    }

    public double getX( GenericHID.Hand hand )
    {
        return controller.getX( hand );
    }

    public double getTrigger( GenericHID.Hand hand )
    {
        return controller.getTriggerAxis( hand );
    }

}
