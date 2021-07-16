package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakePistonControl extends InstantCommand{
    private Intake Intake;
	private boolean extended;

	public IntakePistonControl(Intake Intake, Boolean extended){
        this.Intake = Intake;
		this.extended = extended;
	}
	
    @Override
	public void initialize() {
		addRequirements(Intake);
	}

    @Override
	public void execute() {
		Intake.setPiston(extended);
	}

	@Override
	public void end(boolean interrupted){
	}
 
}
