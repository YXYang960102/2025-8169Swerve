// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGrabberConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.Constants.IntakeConstants.IntakeAngleAction;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class IntakeSubsystem extends SubsystemBase {
  private SparkMax IntakeAngleMotor = new SparkMax(IDConstants.kIntakeAngleMotor, MotorType.kBrushless);
  private SparkMax IntakeMotor = new SparkMax(IDConstants.kIntakeMotor, MotorType.kBrushless);
  private SparkMaxConfig IntakeAngleConfig = new SparkMaxConfig();
  private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();
  private RelativeEncoder IntakeAngleEncoder = IntakeAngleMotor.getEncoder();
  private SparkClosedLoopController IntakePIDController = IntakeAngleMotor.getClosedLoopController();
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    IntakeMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(70);

    IntakeAngleConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(IntakeConstants.kIntakeUpLimit)
        .reverseSoftLimit(IntakeConstants.kIntakeDownLimit);

    IntakeAngleConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeConstants.IntakekP, IntakeConstants.IntakekI, IntakeConstants.IntakekD)
        .iZone(IntakeConstants.IntakekIz)
        .velocityFF(IntakeConstants.IntakekFF)
        .maxOutput(IntakeConstants.IntakekMaxOutput)
        .minOutput(IntakeConstants.IntakekMinOutput);

    IntakeMotor.configure(IntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeAngleMotor.configure(IntakeAngleConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public double getIntakePosition() {
    return IntakeAngleEncoder.getPosition();
  }

  public double getIntakeVelocity() {
    return IntakeAngleEncoder.getVelocity();
  }

  public void setIntakeAction(IntakeAction action) {
    IntakeMotor.set(action.rate);
  }

  public void setAngleAction(IntakeAngleAction action) {
    IntakeAngleMotor.set(action.rate);
  }

  public void setState(IntakeState state) {
    IntakePIDController.setReference(state.position, ControlType.kPosition);
  }

  public void setAngleHold() {
    IntakePIDController.setReference(getIntakePosition(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Position", getIntakePosition());
  }
}
