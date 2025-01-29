// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ref.Reference;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
// import frc.robot.Constants.ElevatorConstants.HeightSet;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax elevatorLMotor = new SparkMax(IDConstants.kElevatorLMotor, MotorType.kBrushless);
  private final SparkMax elevatorRMotor = new SparkMax(IDConstants.kElevatorRMotor, MotorType.kBrushless);
  private final SparkMaxConfig elevatorLConfig = new SparkMaxConfig();
  private final SparkMaxConfig elevatorRConfig = new SparkMaxConfig();
  private final RelativeEncoder elevatorEncoder = elevatorRMotor.getEncoder();
  // private final SparkAbsoluteEncoder elevatorAbsEncoder = elevatorRMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController elevatorPIDController = elevatorRMotor.getClosedLoopController();


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorLConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .follow(elevatorRMotor, true);

    elevatorRConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    // elevatorEncoder.setPosition(elevatorAbsEncoder.getPosition());

    elevatorRMotor.configure(elevatorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorLMotor.configure(elevatorLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRConfig.softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(ElevatorConstants.kUpLimit)
        .reverseSoftLimit(ElevatorConstants.kDefaultLimit);

    elevatorRConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .iZone(ElevatorConstants.kIz)
        .velocityFF(ElevatorConstants.kFF)
        .maxOutput(ElevatorConstants.kMaxOutput)
        .minOutput(ElevatorConstants.kMinOutput);

    // // Initialize the absolute position to relative encoder
    // initializeEncoders();
  }

  // public void initializeEncoders() {
  // // Use the absolute encoder position to set the relative encoder
  // double absolutePosition = elevatorAbsEncoder.getPosition(); // Position in
  // rotations
  // elevatorEncoder.setPosition(absolutePosition * (2 * Math.PI)); // Convert
  // rotations to radians/meters
  // }

  public double getCurrentHeight() {
    // Convert relative encoder position to height in meters
    return elevatorEncoder.getPosition();
  }

  // public double getAbsPosition() {
  //   return elevatorAbsEncoder.getPosition();
  // }

  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  // public void setHeight(double heightMeters) {
  // // Clamp the target height to within soft limits
  // heightMeters = Math.max(ElevatorConstants.kUpLimit, Math.min(heightMeters,
  // ElevatorConstants.kUpLimit));
  // elevatorPIDController.setReference(heightMeters,
  // SparkMax.ControlType.kPosition);
  // }

  public void setDefault() {
    elevatorPIDController.setReference(ElevatorConstants.kDefault, ControlType.kPosition);
  }

  public void setProcessor() {
    elevatorPIDController.setReference(ElevatorConstants.kProcessor, ControlType.kPosition);
  }

  public void setAlgae() {
    elevatorPIDController.setReference(ElevatorConstants.kAlgae, ControlType.kPosition);
  }

  public void setReef() {
    elevatorPIDController.setReference(ElevatorConstants.kReef, ControlType.kPosition);
  }

  public void setCoralStation() {
    elevatorPIDController.setReference(ElevatorConstants.kCoralStation, ControlType.kPosition);
  }

  public void setTop() {
    elevatorPIDController.setReference(ElevatorConstants.kTop, ControlType.kPosition);
  }

  public void ElevatorUP() {
    elevatorRMotor.set(ElevatorConstants.kElevatorMotorRate);
  }

  public void ElevatorDown() {
    elevatorRMotor.set(-ElevatorConstants.kElevatorMotorRate);
  }

  public void ElevatorStop() {
    elevatorRMotor.set(0);
  }

  public void ElevatorHold() {
    elevatorPIDController.setReference(getCurrentHeight(), SparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Relative Position", getCurrentHeight());
    // SmartDashboard.putNumber("Absolute Position ", getAbsPosition());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
  }
}
