package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Controllers.WPI_LazyTalonFX;
import frc.lib.math.Boundaries;
import frc.lib.math.Conversions;
import frc.lib.util.Limelight;
import frc.robot.Constants;
import frc.robot.States;
import frc.robot.States.ShooterStates;

public class DriveTrain extends SubsystemBase {

    private WPI_LazyTalonFX leftParent;
    private WPI_LazyTalonFX leftChild;
    private WPI_LazyTalonFX rightParent;
    private WPI_LazyTalonFX rightChild;

    private DifferentialDrive robotDrive;
    private DifferentialDriveOdometry odometry;
    private SimpleMotorFeedforward driveFF;

    private final TrapezoidProfile.Constraints constraints;
    private final ProfiledPIDController controller;
    private final Limelight Limelight;

    private AHRS gyro;

    // private double previousP = 0;

    public DriveTrain(Vision vision) {
        leftParent = new WPI_LazyTalonFX(Constants.Drive.leftParent);
        leftChild = new WPI_LazyTalonFX(Constants.Drive.leftChild);
        rightParent = new WPI_LazyTalonFX(Constants.Drive.rightParent);
        rightChild = new WPI_LazyTalonFX(Constants.Drive.rightChild);

        leftParent.configPID(Constants.Drive.drivePID);
        rightParent.configPID(Constants.Drive.drivePID);

        leftChild.follow(leftParent);
        rightChild.follow(rightParent);

        gyro = new AHRS();
        zeroGyro();

        robotDrive = new DifferentialDrive(leftParent, rightParent);
        robotDrive.setRightSideInverted(false);
        robotDrive.setSafetyEnabled(false);
        odometry = new DifferentialDriveOdometry(getYaw());
        driveFF = new SimpleMotorFeedforward(Constants.Drive.drivekS / 12, Constants.Drive.drivekV / 12, Constants.Drive.drivekA / 12);

        constraints = new TrapezoidProfile.Constraints(1.75, 1.75);
        controller = new ProfiledPIDController(0.02, 0.0, 0.001, constraints);
        controller.setGoal(0);
        Limelight = vision.limelight;

    }

    /* For standard Teleop Drive */
    public void arcadeDrive(double speed, double rotation, Boolean squaredInput){
        robotDrive.arcadeDrive(speed, rotation, squaredInput);
    }
    
    public void curvDrive(double speed, double rotation, Boolean quickTurn){
        robotDrive.curvatureDrive(speed, rotation, quickTurn);
    }

    /* Used for RamseteController for Following Auto Paths */
    public void setWheelState(double leftSpeed, double rightSpeed){
        double leftDemand = Conversions.MPSToFalcon(leftSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);
        double rightDemand = Conversions.MPSToFalcon(rightSpeed, Constants.Drive.wheelCircumference, Constants.Drive.gearRatio);

        leftParent.set(ControlMode.Velocity, leftDemand, DemandType.ArbitraryFeedForward, driveFF.calculate(leftDemand));
        rightParent.set(ControlMode.Velocity, rightDemand, DemandType.ArbitraryFeedForward, driveFF.calculate(rightDemand));
    }

    public Rotation2d getYaw() {
        double yaw = Boundaries.to360Boundaries(gyro.getAngle());
        return Constants.Drive.invertGyro ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public void resetOdometry(Pose2d pose) {
        leftParent.setSelectedSensorPosition(0);
        rightParent.setSelectedSensorPosition(0);
        odometry.resetPosition(pose, getYaw());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getLeftPos(){
        return Conversions.falconToMeters(
            leftParent.getSelectedSensorPosition(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio
        );
    }

    public double getRightPos(){
        return Conversions.falconToMeters(
            rightParent.getSelectedSensorPosition(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio
        );
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getLeftPos(), getRightPos());
        
        switch(States.shooterState){
            case notCalibrated:
                break;

            case disabled:
                break;
                
            case preShoot:
                robotDrive.arcadeDrive(0.0, controller.calculate(Limelight.getTx().getDegrees()), false);
                break;
        }

        // double mps = SmartDashboard.getNumber("Drive mps", 0);
        // setWheelState(mps, mps);

        // if (previousP != SmartDashboard.getNumber("Drive p", 0)){
        //     previousP = SmartDashboard.getNumber("Drive p", 0);
        //     leftParent.config_kP(0, previousP);
        //     rightParent.config_kP(0, previousP);
        // }

        SmartDashboard.putNumber("left drive", Conversions.falconToMPS(
            leftParent.getSelectedSensorVelocity(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio));

        SmartDashboard.putNumber("right drive", Conversions.falconToMPS(
            rightParent.getSelectedSensorVelocity(), 
            Constants.Drive.wheelCircumference, 
            Constants.Drive.gearRatio));

        SmartDashboard.putNumber("yaw", getYaw().getDegrees());
        SmartDashboard.putNumber("x odo", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y odo", odometry.getPoseMeters().getY());

        SmartDashboard.putNumber("Left Amps", leftParent.getSupplyCurrent());
        SmartDashboard.putNumber("Right Amps", rightParent.getSupplyCurrent());

        SmartDashboard.putNumber("LDrive Temp", leftParent.getTemperature());
        SmartDashboard.putNumber("RDrive Temp", rightParent.getTemperature());


    }
}
