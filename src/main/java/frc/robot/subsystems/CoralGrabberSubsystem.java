// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberState;
import frc.robot.Constants.CoralGrabberConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberAction;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberAngleAction;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberState;

public class CoralGrabberSubsystem extends SubsystemBase {
  private SparkMax coralVortex = new SparkMax(IDConstants.kCoralVortex, MotorType.kBrushless);
  private SparkMax CoralGrabberAngleMotor = new SparkMax(IDConstants.kCoralGrabberAngle, MotorType.kBrushless);
  private SparkMaxConfig coralVortexConfig = new SparkMaxConfig();
  private SparkMaxConfig CoralGrabberAngleConfig = new SparkMaxConfig();
  private RelativeEncoder CoralGrabberAngleEncoder = CoralGrabberAngleMotor.getEncoder();
  private SparkAbsoluteEncoder CoralGrabberAngleAbsEncoder = CoralGrabberAngleMotor.getAbsoluteEncoder();
  private SparkClosedLoopController CoralGrabberAnglePIDController = CoralGrabberAngleMotor.getClosedLoopController();

  private DigitalInput sensor = new DigitalInput(0);

  private boolean coralRevSlowRunning = false;
  public boolean coralBlock = false;

  // private I2C.Port i2cPort = I2C.Port.kOnboard;
  // private ColorSensorV3 coralSensorR = new ColorSensorV3(i2cPort);
  // private ColorSensorV3 coralSensorL = new ColorSensorV3(i2cPort);

  /** Creates a new GrabberSubsystem. */
  public CoralGrabberSubsystem() {

    CoralGrabberAngleEncoder.setPosition(CoralGrabberAngleAbsEncoder.getPosition());
    // coralSensorR.configureProximitySensor(ProximitySensorResolution.kProxRes11bit,
    // ProximitySensorMeasurementRate.kProxRate12ms);

    coralVortexConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(70);

    CoralGrabberAngleConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(CoralGrabberConstants.kCoralUpLimit)
        .reverseSoftLimit(CoralGrabberConstants.kCoralDownLimit);

    CoralGrabberAngleConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CoralGrabberConstants.CoralkP, CoralGrabberConstants.CoralkI, CoralGrabberConstants.CoralkD)
        .iZone(CoralGrabberConstants.CoralkIz)
        .velocityFF(CoralGrabberConstants.CoralkFF)
        .maxOutput(CoralGrabberConstants.CoralkMaxOutput)
        .minOutput(CoralGrabberConstants.CoralkMinOutput);

    coralVortex.configure(coralVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CoralGrabberAngleMotor.configure(CoralGrabberAngleConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // coralSensorL.configureProximitySensor(ProximitySensorResolution.kProxRes11bit,
    // ProximitySensorMeasurementRate.kProxRate25ms);

  }

  public boolean getSensor(){
    return !sensor.get();
  }

  // return Angle Relativ6e Position
  public double getCoralPosition() {
    return CoralGrabberAngleEncoder.getPosition();
  }

  // return Angle Velocity
  public double getCoralVelocity() {
    return CoralGrabberAngleEncoder.getVelocity();
  }

  // return Angle Absolute Position
  public double getCoralAbsPosition() {
    return CoralGrabberAngleAbsEncoder.getPosition();
  }

  public boolean isDefault(){
    return Math.abs(getCoralAbsPosition() - CoralGrabberState.kDefult.position) < 0.04;
  }

  public boolean isSafe(){
    return getCoralAbsPosition() < CoralGrabberState.kSafe.position - 0.01;
  }

  public void setGrabberAction(CoralGrabberAction action) {
    coralVortex.set(action.rate);
  }

  public void setAngleAction(CoralGrabberAngleAction action) {
    CoralGrabberAngleMotor.set(action.rate);
  }

  public void setState(CoralGrabberState state) {
    CoralGrabberAnglePIDController.setReference(state.position, ControlType.kPosition);
  }

  public void setAngleHold() {
    CoralGrabberAnglePIDController.setReference(getCoralAbsPosition(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Coral Grabber Position", getCoralPosition());
    SmartDashboard.putNumber("Coral Grabber Abs Position", getCoralAbsPosition());
    // SmartDashboard.putNumber("Coral Grabber Velocity", getCoralVelocity());
    // SmartDashboard.putBoolean("isGet", RisPass());

    SmartDashboard.putBoolean("Sensor", getSensor());
  }

}