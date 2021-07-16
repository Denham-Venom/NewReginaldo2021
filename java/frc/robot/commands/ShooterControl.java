package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterControl extends CommandBase {
  private Shooter Shooter;
  private double power;

  public ShooterControl(Shooter Shooter, double power) {
    this.Shooter = Shooter;
    this.power = power;
  }

  @Override
  public void initialize() {
    addRequirements(Shooter);
  }

  @Override
  public void execute() {
    Shooter.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    Shooter.setPower(0);
  }
}
