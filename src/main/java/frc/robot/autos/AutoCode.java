package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;
import frc.robot.commands.IndexerControl;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.KickerControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Start directly in front of the goal

public class AutoCode extends SequentialCommandGroup {

  // All units in meters.
  private Trajectory autoTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(                             
          new Translation2d(-2, 0)
        ), 
        new Pose2d(-2, 0, new Rotation2d(0)), 
      Constants.AutoConstants.trajConfig);

  public AutoCode(DriveTrain robotDrive, Kicker Kicker, Indexer Indexer, Intake Intake) {
    addRequirements(robotDrive);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            autoTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            Constants.Drive.driveKinematics,
            (leftspeed, rightspeed) -> robotDrive.setWheelState(leftspeed, rightspeed),
            robotDrive);

    addCommands(
      new InstantCommand(() -> robotDrive.zeroGyro()),
      new InstantCommand(() -> robotDrive.resetOdometry(autoTrajectory.getInitialPose())),
      new InstantCommand(() -> States.shooterState = ShooterStates.preShoot),
      new InstantCommand(() -> {
        // delay
        long start = System.currentTimeMillis();
        long cur;
        do {
          cur = System.currentTimeMillis();
        } while(cur-start < 5000);
      }),
      new ParallelDeadlineGroup(
        // new InstantCommand(() -> {
        //   // timer
        //   long start = System.currentTimeMillis();
        //   long cur;
        //   do {
        //     cur = System.currentTimeMillis();
        //   } while(cur-start < 5000);
        // }),
        new InstantCommand(() -> Timer.delay(1)),
        new KickerControl(Kicker, 1.0),
        new IndexerControl(Indexer, 1.0),
        new IntakeControl(Intake, 1.0)
      ),
      new InstantCommand(() -> States.shooterState = ShooterStates.disabled),
      ramseteCommand
    );
  }

}
