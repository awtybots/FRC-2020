package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller
{

    public JoystickButton btnA, btnX, btnY, btnB, bmpL, bmpR;
    private XboxController controller;
    private double stick_dz, trigger_dz;

    public Controller( int port, double stick_deadzone )
    {
        this(port, stick_deadzone, 0.05); // TODO probs should be 0?
    }

    public Controller( int port, double stick_deadzone, double trigger_deadzone )
    {
        controller = new XboxController(port);
        trigger_dz = trigger_deadzone;
        stick_dz = stick_deadzone;

        btnA = new JoystickButton(controller, Button.kA.value);
        btnB = new JoystickButton(controller, Button.kB.value);
        btnX = new JoystickButton(controller, Button.kX.value);
        btnY = new JoystickButton(controller, Button.kY.value);
        bmpL = new JoystickButton(controller, Button.kBumperLeft.value);
        bmpR = new JoystickButton(controller, Button.kBumperRight.value);
    }

    private double dzone( double x, double dz )
    {
        if(Math.abs(x) < dz)
            return 0;
        else
            return (x - dz * Math.signum(x)) / (1.0 - dz);
    }

    public double getY( GenericHID.Hand hand )
    {
        return dzone(controller.getY( hand ), stick_dz);
    }

    public double getX( GenericHID.Hand hand )
    {
        return dzone(controller.getX( hand ), stick_dz);
    }

    public double getTrigger( GenericHID.Hand hand )
    {
        return dzone(controller.getTriggerAxis( hand ), trigger_dz);
    }

}
