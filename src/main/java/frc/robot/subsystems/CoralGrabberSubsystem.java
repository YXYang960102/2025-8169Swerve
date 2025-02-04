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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.IDConstants;

public class CoralGrabberSubsystem extends SubsystemBase {
  private SparkMax coralVortex = new SparkMax(IDConstants.kCoralVortex, MotorType.kBrushless);
  private SparkMax CoralGrabberAngleMotor = new SparkMax(IDConstants.kCoralGrabberAngle, MotorType.kBrushless);
  private SparkMaxConfig coralVortexConfig = new SparkMaxConfig();
  private SparkMaxConfig CoralGrabberAngleConfig = new SparkMaxConfig();
  private RelativeEncoder CoralGrabberAngleEncoder = CoralGrabberAngleMotor.getEncoder();
  private SparkAbsoluteEncoder CoralGrabberAngleAbsEncoder = CoralGrabberAngleMotor.getAbsoluteEncoder();
  private SparkClosedLoopController CoralGrabberAnglePIDController = CoralGrabberAngleMotor.getClosedLoopController();
  
  
  /** Creates a new GrabberSubsystem. */
  public CoralGrabberSubsystem() {

    CoralGrabberAngleEncoder.setPosition(CoralGrabberAngleAbsEncoder.getPosition());
    // algaeVortex.configure(algaeVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralVortexConfig
    .idleMode(IdleMode.kCoast)
    .inverted(false)
    .smartCurrentLimit(70);


    CoralGrabberAngleConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(AngleConstants.CoralkP, AngleConstants.CoralkI, AngleConstants.CoralkD)
    .iZone(AngleConstants.CoralkIz)
    .velocityFF(AngleConstants.CoralkFF)
    .maxOutput(AngleConstants.CoralkMaxOutput)
    .minOutput(AngleConstants.CoralkMinOutput);
    
    CoralGrabberAngleConfig.softLimit
    .forwardSoftLimitEnabled(false)
    .reverseSoftLimitEnabled(false)
    .forwardSoftLimit(AngleConstants.kCoralUpLimit)
    .reverseSoftLimit(AngleConstants.kCoralDownLimit);

    coralVortex.configure(coralVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CoralGrabberAngleMotor.configure(CoralGrabberAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // algaeVortexConfig
    // .idleMode(IdleMode.kBrake)
    // .inverted(false)
    // .secondaryCurrentLimit(70);


  }

  //return Angle Relative Position
  public double getCoralPosition() {
    return CoralGrabberAngleEncoder.getPosition();
  }

  //return Angle Velocity
  public double getCoralVelocity() {
    return CoralGrabberAngleEncoder.getVelocity();
  }

  //return Angle Absolute Position
  public double getCoralAbsPosition() {
    return CoralGrabberAngleAbsEncoder.getPosition();
  }

  public void setDefultPosition() {
    CoralGrabberAnglePIDController.setReference(AngleConstants.kCoralDefultPosition, ControlType.kPosition);
  }

  // public void setCoralStationPosition() {
  //   CoralGrabberAnglePIDController.setReference(AngleConstants.kCoralStationPosition, ControlType.kPosition);
  // }

  public void setCoralTopPosition() {
    CoralGrabberAnglePIDController.setReference(AngleConstants.kCoralTopPosition, ControlType.kPosition);
  }

  public void  CoralGrabberAngleUP() {
    CoralGrabberAngleMotor.set(AngleConstants.kCoralAngleMotorRate);
  }

  public void  CoralGrabberAngleDown() {
    CoralGrabberAngleMotor.set(-AngleConstants.kCoralAngleMotorRate);
  }

  public void  CoralGrabberAngleStop() {
    CoralGrabberAngleMotor.set(0);
  }

  public void  CoralGrabberAngleHold() {
    CoralGrabberAnglePIDController.setReference(getCoralAbsPosition(), ControlType.kPosition);
  }


  public void getCoral() {
    coralVortex.set(GrabberConstants.CoralmotorRate);
  }

  public void putCoral() {
    coralVortex.set(-GrabberConstants.CoralmotorRate);
  }

  public void StopMotor() {
    coralVortex.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}
