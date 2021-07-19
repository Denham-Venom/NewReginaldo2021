package frc.lib.Controllers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.lib.math.PIDGains;

/**
 * Thin Spark Max wrapper to make setup easier, 
 * and automatically initalize the internal CANEncoder and PIDController.
 */
public class LazySparkMAX extends CANSparkMax {
    protected CANPIDController pidController;
    protected CANEncoder encoder;

    /**
     * Config a Spark Max using sparkConstants.
     * 
     * @param sparkConstants
     */
    public LazySparkMAX(SparkConstants sparkConstants) {
        super(sparkConstants.deviceId, sparkConstants.motorType);
        super.restoreFactoryDefaults();
        super.setSmartCurrentLimit(sparkConstants.smartCurrentLimit);
        super.enableVoltageCompensation(12);
        super.setIdleMode(sparkConstants.idleMode);
        super.setInverted(sparkConstants.inverted);
        super.burnFlash();

        pidController = super.getPIDController();
        encoder = super.getEncoder();

        encoder.setPosition(0);
    }
    
    public void set(ControlType type, double setpoint) {
        pidController.setReference(setpoint, type);
    }
    
    /**
     * Config PID Gains and Peak Outputs using PIDGains
     * @param pidGains
     */
    public void configPID(PIDGains pidGains){
        pidController.setP(pidGains.kP);
        pidController.setI(pidGains.kI);
        pidController.setD(pidGains.kD);
        pidController.setFF(pidGains.kFF);
        pidController.setOutputRange(pidGains.kMaxReverse, pidGains.kMaxForward);
    }

    /**
     * Returns PIDGains and configured Peak Outputs from internal PIDController
     * @return
     */
    public PIDGains getPIDGains(){
        PIDGains pidGains = new PIDGains(
            pidController.getP(), 
            pidController.getI(), 
            pidController.getD(), 
            pidController.getFF(), 
            pidController.getOutputMax(), 
            pidController.getOutputMin()
        );
        return pidGains;
    }

    /**
     * Set the internal pid controllers feedback device.
     * @param sensor
     */
    public void setFeedbackDevice(CANSensor sensor) {
        pidController.setFeedbackDevice(sensor);
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void setPosition(double position){
        encoder.setPosition(position);
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    /**
     * Set conversion factor of encoder, so that position is in meters and velocity is in MPS.
     * @param circumference Circumference of wheel in meters
     * @param gearRatio Reduction to wheel. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorMeters(double circumference, double gearRatio){
        encoder.setPositionConversionFactor(circumference / gearRatio);
        encoder.setVelocityConversionFactor((circumference / gearRatio) / 60.0);
    }

    /**
     * Set conversion factor of encoder, so that position is in rotations and velocity is in RPM
     * @param gearRatio Reduction to mech. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorRotations(double gearRatio){
        encoder.setPositionConversionFactor(1 / gearRatio);
        encoder.setVelocityConversionFactor(1 / gearRatio);
    }

    /**
     * Set conversion factor of encoder, so that position is in degrees and velocity is in degrees/min
     * @param gearRatio Reduction to mech. For example "15" if on a 15:1 reduction.
     */
    public void setConversionFactorDegrees(double gearRatio){
        encoder.setPositionConversionFactor(360 / gearRatio);
        encoder.setVelocityConversionFactor(360 / gearRatio);
    }



    
}