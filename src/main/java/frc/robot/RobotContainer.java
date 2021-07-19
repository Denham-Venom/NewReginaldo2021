package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.States.ShooterStates;
import frc.robot.autos.TrenchRun;
import frc.robot.commands.IndexerControl;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.IntakePistonControl;
import frc.robot.commands.KickerControl;
import frc.robot.commands.teleopDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class RobotContainer {
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Driver Controls */
    // private final int forwardAxis = XboxController.Axis.kLeftY.value;
    private final int forwardAxis = XboxController.Axis.kRightTrigger.value;
    private final int reverseAxis = XboxController.Axis.kLeftTrigger.value;
    private final int rotationAxis = XboxController.Axis.kLeftX.value;
    private final int quickTurnButton = XboxController.Button.kX.value;

    /* Driver Buttons */
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kBumperRight.value);
    private final JoystickButton outakeButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final POVButton intakeExtendButton = new POVButton(driver, 180);
    private final POVButton intakeRetractButton = new POVButton(driver, 0);
    private final JoystickButton zeroGyroButton = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Operator Buttons */
    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton shooterActivateButton = new JoystickButton(operator, XboxController.Button.kBumperRight.value);
    private final JoystickButton shooterDeactivateButton = new JoystickButton(operator, XboxController.Button.kBumperLeft.value);
    private final JoystickButton zeroShooterAngleButton = new JoystickButton(operator, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Vision Vision = new Vision();
    private final DriveTrain robotDrive = new DriveTrain(Vision);
    private final Intake Intake = new Intake();
    private final Indexer Indexer = new Indexer();
    private final Kicker Kicker = new Kicker();
    private final Shooter Shooter = new Shooter(Vision);

    public RobotContainer() {
    /* Teleop Drive */
    robotDrive.setDefaultCommand(
        new teleopDrive(
            robotDrive, 
            Vision, 
            () -> (driver.getRawAxis(forwardAxis) - driver.getRawAxis(reverseAxis)), 
            () -> driver.getRawAxis(rotationAxis), 
            () -> driver.getRawButton(quickTurnButton)
        )
    );

    configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Intake */
        intakeButton.whileHeld(new IntakeControl(Intake, 1.0));
        outakeButton.whileHeld(new IntakeControl(Intake, -1.0));
        intakeExtendButton.whenPressed(new IntakePistonControl(Intake, true));
        intakeRetractButton.whenPressed(new IntakePistonControl(Intake, false));

        /* Shooting */
        shootButton.whileHeld(
            new ParallelCommandGroup(
                new KickerControl(Kicker, 1.0), 
                new IndexerControl(Indexer, 1.0),
                new IntakeControl(Intake, 1.0)            
            )
        );
        shooterActivateButton.whenPressed(new InstantCommand(() -> activate_Shooter()));
        shooterDeactivateButton.whenPressed(new InstantCommand(() -> deactivate_Shooter()));
        zeroShooterAngleButton.whenPressed(new InstantCommand(() -> zeroShooterAngle()));

        /* DriveTrain */
        zeroGyroButton.whenPressed(new InstantCommand(() -> zeroGyro()));

    }

    /* Shooter States */
    private void activate_Shooter() {
        States.shooterState = ShooterStates.preShoot;
    }

    private void deactivate_Shooter() {
        States.shooterState = ShooterStates.disabled;
    }

    private void zeroGyro() {
        robotDrive.zeroGyro();
    }

    private void zeroShooterAngle() {
        Shooter.resetShooterAngle();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.*
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new TrenchRun(robotDrive).andThen(() -> robotDrive.arcadeDrive(0, 0, false));
    }
}
