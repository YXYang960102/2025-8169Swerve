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
  // private final SparkAbsoluteEncoder elevatorAbsEncoder =
  // elevatorRMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController elevatorPIDController = elevatorRMotor.getClosedLoopController();

  // public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

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

    // kP = ElevatorConstants.kP;
    // kI = ElevatorConstants.kI;
    // kD = ElevatorConstants.kD;
    // kIz = ElevatorConstants.kIz;
    // kFF = ElevatorConstants.kFF;
    // kMaxOutput = ElevatorConstants.kMaxOutput;
    // kMinOutput = ElevatorConstants.kMinOutput;

    elevatorRConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(ElevatorConstants.kUpLimit)
        .reverseSoftLimit(ElevatorConstants.kDefaultLimit);

    elevatorRConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)
        .iZone(ElevatorConstants.kIz)
        .velocityFF(ElevatorConstants.kFF)
        .maxOutput(ElevatorConstants.kMaxOutput)
        .minOutput(ElevatorConstants.kMinOutput);

    elevatorRMotor.configure(elevatorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorLMotor.configure(elevatorLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

  }

  public double getCurrentHeight() {
    // Convert relative encoder position to height in meters
    return elevatorEncoder.getPosition();
  }

  public double getVelocity() {
    return elevatorEncoder.getVelocity();
  }

  public void setDefault() {
    elevatorPIDController.setReference(ElevatorConstants.kDefault, ControlType.kPosition);
  }

  public void setL1() {
    elevatorPIDController.setReference(ElevatorConstants.kL1, ControlType.kPosition);
  }

  public void setL2() {
    elevatorPIDController.setReference(ElevatorConstants.kL2, ControlType.kPosition);
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
    elevatorPIDController.setReference(getCurrentHeight(), ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Position", getCurrentHeight());
    // SmartDashboard.putNumber("Elevator Velocity", getVelocity());

    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("Elevator P Gain", 0);
    // double i = SmartDashboard.getNumber("Elevator I Gain", 0);
    // double d = SmartDashboard.getNumber("Elevator D Gain", 0);
    // double iz = SmartDashboard.getNumber("Elevator I Zone", 0);
    // double ff = SmartDashboard.getNumber("Elevator Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Elevator Max Output", 0);
    // double min = SmartDashboard.getNumber("Elevator Min Output", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to
    // // controller
    // if ((p != kP)) {
    //   elevatorRConfig.closedLoop.p(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   elevatorRConfig.closedLoop.i(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   elevatorRConfig.closedLoop.d(d);
    //   kD = d;
    // }
    // if ((iz != kIz)) {
    //   elevatorRConfig.closedLoop.iZone(iz);
    //   kIz = iz;
    // }
    // if ((ff != kFF)) {
    //   elevatorRConfig.closedLoop.velocityFF(ff);
    //   kFF = ff;
    // }
    // if ((max != kMaxOutput) || (min != kMinOutput)) {
    //   elevatorRConfig.closedLoop.outputRange(min, max);
    //   kMinOutput = min;
    //   kMaxOutput = max;
    // }
  }
}
