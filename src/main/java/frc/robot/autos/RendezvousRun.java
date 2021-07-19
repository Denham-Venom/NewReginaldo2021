// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RendezvousRun extends SequentialCommandGroup {

  // All units in meters.
  private Trajectory rendezvousTrajectory = 
    TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(                             
          new Translation2d(0, 0 ) // while running intake
        ), 
        new Pose2d(0, 0, new Rotation2d(0)), 
      Constants.AutoConstants.trajConfig);

  public RendezvousRun(DriveTrain robotDrive) {
    addRequirements(robotDrive);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            rendezvousTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            Constants.Drive.driveKinematics,
            (leftspeed, rightspeed) -> robotDrive.setWheelState(leftspeed, rightspeed),
            robotDrive);

    addCommands(
      new InstantCommand(() -> robotDrive.zeroGyro()),
      // aim and shoot
      new InstantCommand(() -> robotDrive.resetOdometry(rendezvousTrajectory.getInitialPose())),
      ramseteCommand
      // aim and shoot
    );
  }

}
