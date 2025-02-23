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

import frc.robot.Constants.AlgaeGrabberConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAction;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAngleAction;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberState;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberState;

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

    AlgaeGrabberAngleConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeGrabberConstants.kAlgaeUpLimit)
        .reverseSoftLimit(AlgaeGrabberConstants.kAlgaeDownLimit);

    AlgaeGrabberAngleConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(AlgaeGrabberConstants.AlgaekP, AlgaeGrabberConstants.AlgaekI, AlgaeGrabberConstants.AlgaekD)
        .iZone(AlgaeGrabberConstants.AlgaekIz)
        .velocityFF(AlgaeGrabberConstants.AlgaekFF)
        .maxOutput(AlgaeGrabberConstants.AlgaekMaxOutput)
        .minOutput(AlgaeGrabberConstants.AlgaekMinOutput);


    AlgaeGrabberAngleMotor.configure(AlgaeGrabberAngleConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    algaeVortex.configure(algaeVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // return Angle Relative Position
  public double getAlgaePosition() {
    return AlgaeGrabberAngleEncoder.getPosition();
  }

  // return Angle Velocity
  public double getAlgaeVelocity() {
    return AlgaeGrabberAngleEncoder.getVelocity();
  }

  // return Angle Absolute Position
  public double getAlgaeAbsPosition() {
    return AlgaeGrabberAngleAbsEncoder.getPosition();
  }

  public boolean isSafe(){
    return getAlgaeAbsPosition() < AlgaeGrabberState.kSafe.position + 0.02;
  }

  public void setGrabberAction(AlgaeGrabberAction action) {
    algaeVortex.set(action.rate);
  }

  public void setAngleAction(AlgaeGrabberAngleAction action) {
    AlgaeGrabberAngleMotor.set(action.rate);
  }

  public void setState(AlgaeGrabberState state) {
    AlgaeGrabberAnglePIDController.setReference(state.position, ControlType.kPosition);
  }

  public void setAngleHold() {
    AlgaeGrabberAnglePIDController.setReference(getAlgaeAbsPosition(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Algae Position", getAlgaePosition());
    SmartDashboard.putNumber("Algae Abs Position", getAlgaeAbsPosition());
    // SmartDashboard.putNumber("Algae Velocity", getAlgaeVelocity());
  }

}
