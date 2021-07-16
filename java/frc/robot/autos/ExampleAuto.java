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


public class ExampleAuto extends SequentialCommandGroup {

  // An example trajectory.  All units in meters.
  private Trajectory exampleTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), // Start at the origin facing the +X direction
        List.of(                              // Pass through these two interior waypoints, making an 's' curve path
          new Translation2d(2, 2), 
          new Translation2d(3, -2)
        ), 
        new Pose2d(5, 2, new Rotation2d(0)), // End 3 meters straight ahead of where we started, facing forward
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
      new InstantCommand(() -> robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
      ramseteCommand
    );
  }

}
