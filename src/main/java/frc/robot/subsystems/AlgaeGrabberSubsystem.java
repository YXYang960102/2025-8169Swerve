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
import frc.robot.commands.Angle.AngleAuto;

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

  public void setDefultPosition() {
    AlgaeGrabberAnglePIDController.setReference(AlgaeGrabberConstants.kAlgaeDefultPosition, ControlType.kPosition);
  }

  public void setAlgaeL2Position() {
    AlgaeGrabberAnglePIDController.setReference(AlgaeGrabberConstants.kGetL2AlgaePosition, ControlType.kPosition);
  }

  public void setAlgaeL3Position() {
    AlgaeGrabberAnglePIDController.setReference(AlgaeGrabberConstants.kGetL3AlgaePosition, ControlType.kPosition);
  }

  public void setAlgaePutProPosition() {
    AlgaeGrabberAnglePIDController.setReference(AlgaeGrabberConstants.kAlgaePutProPosition, ControlType.kPosition);
  }

  public void AlgaeGrabberAngleHold() {
    AlgaeGrabberAnglePIDController.setReference(getAlgaeAbsPosition(), ControlType.kPosition);
  }

  public void AlgaeGrabberAngleUP() {
    AlgaeGrabberAngleMotor.set(AlgaeGrabberConstants.kAlgaeAngleMotorRate);
  }

  public void AlgaeGrabberAngleDown() {
    AlgaeGrabberAngleMotor.set(-AlgaeGrabberConstants.kAlgaeAngleMotorRate);
  }

  public void getAlgae() {
    algaeVortex.set(AlgaeGrabberConstants.AlgaeMotorRate);

  }

  public void putAlgae() {
    algaeVortex.set(-AlgaeGrabberConstants.AlgaeMotorRate);

  }

  public void AlgaeMotorStop() {
    algaeVortex.set(0);
  }

  public void AlgaeAngleMotorStop() {
    AlgaeGrabberAngleMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Algae Position", getAlgaePosition());
    SmartDashboard.putNumber("Algae Abs Position", getAlgaeAbsPosition());
    // SmartDashboard.putNumber("Algae Velocity", getAlgaeVelocity());
  }


}
