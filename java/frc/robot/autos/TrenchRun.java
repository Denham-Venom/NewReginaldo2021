package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Start directly in front of the goal

public class TrenchRun extends SequentialCommandGroup {

  // All units in meters.
  private Trajectory trenchTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(                             
          new Translation2d(1, 2), 
          new Translation2d(1, -5.5) // while running intake
        ), 
        new Pose2d(1, 0, new Rotation2d(0)), 
      Constants.AutoConstants.trajConfig);

  public ExampleAuto(DriveTrain robotDrive) {
    addRequirements(robotDrive);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trenchTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            Constants.Drive.driveKinematics,
            (leftspeed, rightspeed) -> robotDrive.setWheelState(leftspeed, rightspeed),
            robotDrive);

    addCommands(
      new InstantCommand(() -> robotDrive.zeroGyro()),
      // aim and shoot
      new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
      ramseteCommand
      // aim and shoot
    );
  }

}

// need aim and shoot sequential command

// Big Square Run
// put into new squential command group later

// start in middle of the line

public class RendezvousRun extends SequentialCommandGroup {

  // All units in meters.
  private Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(                             
          new Translation2d(1, 2), 
          new Translation2d(1, -5.5) // while running intake
        ), 
        new Pose2d(1, 0, new Rotation2d(0)), 
      Constants.AutoConstants.trajConfig);

  public ExampleAuto(DriveTrain robotDrive) {
    addRequirements(robotDrive);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            Constants.Drive.driveKinematics,
            (leftspeed, rightspeed) -> robotDrive.setWheelState(leftspeed, rightspeed),
            robotDrive);

    addCommands(
      new InstantCommand(() -> robotDrive.zeroGyro()),
      // aim and shoot
      new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
      ramseteCommand
      // aim and shoot
    );
  }

}