package frc.robot.commands;

import frc.lib.util.Limelight;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class teleopDrive extends CommandBase {
    private final DriveTrain subsystem;
    private final Limelight Limelight;

    DoubleSupplier throttle;
    DoubleSupplier rotation;
    BooleanSupplier quickTurn;

    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;

    public teleopDrive(DriveTrain subsystem, Vision Vision, DoubleSupplier throttle, DoubleSupplier rotation, BooleanSupplier quickTurn) {
        subsystem = subsystem;
        addRequirements(subsystem);
        Limelight = Vision.limelight;

        this.throttle = throttle;
        this.rotation = rotation;
        this.quickTurn = quickTurn;

        constraints = new TrapezoidProfile.Constraints(1.75, 1.75);
        controller = new ProfiledPIDController(0.08, 0.05, 0.005, constraints);
        controller.setGoal(0);
    }

    @Override
    public void execute() {
        if (States.shooterState == ShooterStates.preShoot && Limelight.hasTarget()){
            subsystem.arcadeDrive(throttle.getAsDouble(), controller.calculate(Limelight.getTx().getDegrees()), true);
        } else{
            subsystem.curvDrive(throttle.getAsDouble(), rotation.getAsDouble(), quickTurn.getAsBoolean());
        }
    }
}
