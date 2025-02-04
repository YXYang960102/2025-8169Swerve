// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AngleConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.AngleConstants.AngleState;
import frc.robot.Constants.GrabberConstants;
import frc.robot.commands.Grabber.AngleAuto;

public class AlgaeGrabberSubsystem extends SubsystemBase {
  private SparkMax AlgaeGrabberAngleMotor = new SparkMax(IDConstants.kAlgaeGrabberAngle, MotorType.kBrushless);
  private SparkMax algaeVortex = new SparkMax(IDConstants.kAlgaeVortex, MotorType.kBrushless);
  private SparkMaxConfig AlgaeGrabberAngleConfig = new SparkMaxConfig();
  private SparkMaxConfig algaeVortexConfig = new SparkMaxConfig();
  private RelativeEncoder AlgaeGrabberAngleEncoder = AlgaeGrabberAngleMotor.getEncoder();
  private SparkAbsoluteEncoder AlgaeGrabberAngleAbsEncoder = AlgaeGrabberAngleMotor.getAbsoluteEncoder();
  private SparkClosedLoopController AlgaeGrabberAnglePIDController = AlgaeGrabberAngleMotor.getClosedLoopController();
  /** Creates a new GrabberAngleSubsystem. */
  public AlgaeGrabberSubsystem() {

    AlgaeGrabberAngleEncoder.setPosition(AlgaeGrabberAngleAbsEncoder.getPosition());


    algaeVortexConfig
     .idleMode(IdleMode.kBrake)
     .inverted(false)
     .secondaryCurrentLimit(70);

    AlgaeGrabberAngleConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(AngleConstants.AlgaekP, AngleConstants.AlgaekI, AngleConstants.AlgaekD)
    .iZone(AngleConstants.AlgaekIz)
    .velocityFF(AngleConstants.AlgaekFF)
    .maxOutput(AngleConstants.AlgaekMaxOutput)
    .minOutput(AngleConstants.AlgaekMinOutput);
    
    AlgaeGrabberAngleConfig.softLimit
    .forwardSoftLimitEnabled(false)
    .reverseSoftLimitEnabled(false)
    .forwardSoftLimit(AngleConstants.kAlgaeUpLimit)
    .reverseSoftLimit(AngleConstants.kAlgaeDownLimit);

    AlgaeGrabberAngleMotor.configure(AlgaeGrabberAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algaeVortex.configure(algaeVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void setDefultPosition() {
    AlgaeGrabberAnglePIDController.setReference(AngleConstants.kAlgaeDefultPosition, ControlType.kPosition);
  }

  //return Angle Relative Position
  public double getAlgaePosition() {
    return AlgaeGrabberAngleEncoder.getPosition();
  }

  //return Angle Velocity
  public double getAlgaeVelocity() {
    return AlgaeGrabberAngleEncoder.getVelocity();
  }

  //return Angle Absolute Position
  public double getAlgaeAbsPosition() {
    return AlgaeGrabberAngleAbsEncoder.getPosition();
  }

  

  // public void setProcessorPosition() {
  //   AlgaeGrabberAnglePIDController.setReference(AngleConstants.kProcessorPosition, ControlType.kPosition);
  // }

  // public void setReefPosition() {
  //   AlgaeGrabberAnglePIDController.setReference(AngleConstants.kReefPosition, ControlType.kPosition);
  // }

  
  public void setAlgaeTopPosition() {
    AlgaeGrabberAnglePIDController.setReference(AngleConstants.kAlgaeTopPosition, ControlType.kPosition);
  }

 

  public void  AlgaeGrabberAngleHold() {
    AlgaeGrabberAnglePIDController.setReference(getAlgaeAbsPosition(), ControlType.kPosition);
  }

  public void  AlgaeGrabberAngleUP() {
    AlgaeGrabberAngleMotor.set(AngleConstants.kAlgaeAngleMotorRate);
  }

  public void  AlgaeGrabberAngleDown() {
    AlgaeGrabberAngleMotor.set(-AngleConstants.kAlgaeAngleMotorRate);
  }

  public void getAlgae() {
    algaeVortex.set(GrabberConstants.AlgaeMotorRate);
    
  }

  public void putAlgae() {
    algaeVortex.set(-GrabberConstants.AlgaeMotorRate);
    
  }

  public void AlgaeMotorStop() {
    algaeVortex.set(0);
  }

  public void AlgaeAngleMotorStop() {
    AlgaeGrabberAngleMotor.set(0);
  }

}
