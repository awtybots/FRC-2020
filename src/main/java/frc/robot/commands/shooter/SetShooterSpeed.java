package frc.robot.commands.shooter;

import static frc.robot.Robot.indexerTowerSubsystem;
import static frc.robot.Robot.shooterSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetShooterSpeed extends CommandBase {
  private double rpm;
  private boolean towerToggled;
  private boolean autoIndex;

  public SetShooterSpeed(double rpm_) {
    this(rpm_, false);
  }

  public SetShooterSpeed(double rpm_, boolean autoIndex) {
    this.rpm = rpm_;
    this.autoIndex = autoIndex;
    towerToggled = false;
  }

  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelGoalVelocity(rpm);
  }

  @Override
  public void execute() {
    shooterSubsystem.flywheelPID();
    if (autoIndex && (!towerToggled) && shooterSubsystem.isVelocityAtGoal()) {
      indexerTowerSubsystem.toggle(true);
      towerToggled = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFlywheelGoalVelocity(0);
    if (towerToggled) indexerTowerSubsystem.toggle(false);
  }
}
