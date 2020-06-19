package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {

  private static final double DEADZONE_STICK = 0.2;
  private static final double DEADZONE_TRIGGER = 0.1;

  public JoystickButton btnA, btnX, btnY, btnB, bmpL, bmpR;
  private XboxController controller;

  public Controller(int port) {
    controller = new XboxController(port);

    btnA = new JoystickButton(controller, Button.kA.value);
    btnB = new JoystickButton(controller, Button.kB.value);
    btnX = new JoystickButton(controller, Button.kX.value);
    btnY = new JoystickButton(controller, Button.kY.value);
    bmpL = new JoystickButton(controller, Button.kBumperLeft.value);
    bmpR = new JoystickButton(controller, Button.kBumperRight.value);
  }

  private double deadzone(double x, double dz) {
    if (Math.abs(x) < dz) return 0;
    else return (x - dz * Math.signum(x)) / (1.0 - dz);
  }

  public double getY(GenericHID.Hand hand) {
    return deadzone(controller.getY(hand), DEADZONE_STICK);
  }

  public double getX(GenericHID.Hand hand) {
    return deadzone(controller.getX(hand), DEADZONE_STICK);
  }

  public double getTrigger(GenericHID.Hand hand) {
    return deadzone(controller.getTriggerAxis(hand), DEADZONE_TRIGGER);
  }
}
