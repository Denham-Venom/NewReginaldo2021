package frc.robot.commands;

import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerControl extends CommandBase {
  private Indexer Indexer;
  private double power;

  public IndexerControl(Indexer Indexer, double power) {
    this.Indexer = Indexer;
    this.power = power;
  }

  @Override
  public void initialize() {
    addRequirements(Indexer);
  }

  @Override
  public void execute() {
    Indexer.setPower(power);
  }

  @Override
  public void end(boolean interrupted) {
    Indexer.setPower(0);
  }
}
