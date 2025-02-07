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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGrabberConstants;
// import frc.robot.Constants.CoralGrabberConstants;
// import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.IDConstants;

public class CoralGrabberSubsystem extends SubsystemBase {
  private SparkFlex coralVortex = new SparkFlex(IDConstants.kCoralVortex, MotorType.kBrushless);
  private SparkMax CoralGrabberAngleMotor = new SparkMax(IDConstants.kCoralGrabberAngle, MotorType.kBrushless);
  private SparkFlexConfig coralVortexConfig = new SparkFlexConfig();
  private SparkMaxConfig CoralGrabberAngleConfig = new SparkMaxConfig();
  private RelativeEncoder CoralGrabberAngleEncoder = CoralGrabberAngleMotor.getEncoder();
  private SparkAbsoluteEncoder CoralGrabberAngleAbsEncoder = CoralGrabberAngleMotor.getAbsoluteEncoder();
  private SparkClosedLoopController CoralGrabberAnglePIDController = CoralGrabberAngleMotor.getClosedLoopController();

  /** Creates a new GrabberSubsystem. */
  public CoralGrabberSubsystem() {

    CoralGrabberAngleEncoder.setPosition(CoralGrabberAngleAbsEncoder.getPosition());

    coralVortexConfig
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(70);

    CoralGrabberAngleConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CoralGrabberConstants.CoralkP, CoralGrabberConstants.CoralkI, CoralGrabberConstants.CoralkD)
        .iZone(CoralGrabberConstants.CoralkIz)
        .velocityFF(CoralGrabberConstants.CoralkFF)
        .maxOutput(CoralGrabberConstants.CoralkMaxOutput)
        .minOutput(CoralGrabberConstants.CoralkMinOutput);

    CoralGrabberAngleConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(CoralGrabberConstants.kCoralUpLimit)
        .reverseSoftLimit(CoralGrabberConstants.kCoralDownLimit);

    coralVortex.configure(coralVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CoralGrabberAngleMotor.configure(CoralGrabberAngleConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


  }

  // return Angle Relative Position
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

  public void setDefultPosition() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kCoralDefultPosition, ControlType.kPosition);
  }

  public void setCoralTopPosition() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kCoralTopPosition, ControlType.kPosition);
  }

  public void CoralGrabberAngleUP() {
    CoralGrabberAngleMotor.set(CoralGrabberConstants.kCoralAngleMotorRate);
  }

  public void CoralGrabberAngleDown() {
    CoralGrabberAngleMotor.set(-CoralGrabberConstants.kCoralAngleMotorRate);
  }

  public void CoralGrabberAngleStop() {
    CoralGrabberAngleMotor.set(0);
  }

  public void CoralGrabberAngleHold() {
    CoralGrabberAnglePIDController.setReference(getCoralAbsPosition(), ControlType.kPosition);
  }

  public void getCoral() {
    coralVortex.set(CoralGrabberConstants.CoralmotorRate);
  }

  public void putCoral() {
    coralVortex.set(-CoralGrabberConstants.CoralmotorRate);
  }

  public void StopMotor() {
    coralVortex.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Grabber Position", getCoralPosition());
    SmartDashboard.putNumber("Coral Grabber Abs Position", getCoralAbsPosition());
    SmartDashboard.putNumber("Coral Grabber Velocity", getCoralVelocity());
  }

}
