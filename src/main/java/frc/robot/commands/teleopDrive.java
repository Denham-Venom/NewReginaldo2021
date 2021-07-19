package frc.robot.commands;

import frc.lib.util.Limelight;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class teleopDrive extends CommandBase {
    private final DriveTrain subsystem;
    private final Limelight Limelight;

    DoubleSupplier throttle;
    DoubleSupplier rotation;
    BooleanSupplier quickTurn;
    
    double throttleDouble;
    double rotationDouble;

    public teleopDrive(DriveTrain subsystem, Vision Vision, DoubleSupplier throttle, DoubleSupplier rotation, BooleanSupplier quickTurn) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
        Limelight = Vision.limelight;

        this.throttle = throttle;
        this.rotation = rotation;
        this.quickTurn = quickTurn;
    }

    @Override
    public void execute() {
        /* Deadbands */
        throttleDouble = (Math.abs(throttle.getAsDouble()) < 0.1) ? 0 : throttle.getAsDouble();
        rotationDouble = (Math.abs(rotation.getAsDouble()) < 0.1) ? 0 : rotation.getAsDouble();

        if (States.shooterState != ShooterStates.preShoot){
            subsystem.curvDrive(throttle.getAsDouble(), rotation.getAsDouble(), quickTurn.getAsBoolean());
        }
    }
}
