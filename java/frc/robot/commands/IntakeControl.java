package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeControl extends CommandBase {
  private Intake Intake;
  private double power;

  public IntakeControl(Intake Intake, double power) {
    this.Intake = Intake;
    this.power = power;
  }

  @Override
  public void initialize() {
    addRequirements(Intake);
  }

  @Override
  public void execute() {
    Intake.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    Intake.setPower(0);
  }
}
