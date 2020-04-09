package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {

    public XboxController cntrl;
    public JoystickButton btnA, btnX, btnY, btnB, bmpL, bmpR;

    public Controller( int port ) {
        cntrl = new XboxController(port);

        btnA = new JoystickButton(cntrl, Button.kA.value);
        btnB = new JoystickButton(cntrl, Button.kB.value);
        btnX = new JoystickButton(cntrl, Button.kX.value);
        btnY = new JoystickButton(cntrl, Button.kY.value);
        bmpL = new JoystickButton(cntrl, Button.kBumperLeft.value);
        bmpR = new JoystickButton(cntrl, Button.kBumperRight.value);
    }

}
