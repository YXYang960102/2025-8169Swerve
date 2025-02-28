// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorAction;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorLMotor = new TalonFX(IDConstants.kElevatorLMotor);
  private final TalonFX elevatorRMotor = new TalonFX(IDConstants.kElevatorRMotor);
  private final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    // Configure left motor to follow the right motor
    elevatorLMotor.setControl(new Follower(elevatorRMotor.getDeviceID(), false));
    
    // Configure soft limits
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    elevatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.kUpLimit;
    elevatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.kDefaultLimit;

    // Configure PID values
    elevatorConfig.Slot0.kP = ElevatorConstants.kP;
    elevatorConfig.Slot0.kI = ElevatorConstants.kI;
    elevatorConfig.Slot0.kD = ElevatorConstants.kD;
     
    elevatorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;

    // Apply configurations
    elevatorRMotor.getConfigurator().apply(elevatorConfig);
    elevatorLMotor.getConfigurator().apply(elevatorConfig);

    elevatorRMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorLMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public double getCurrentHeight() {
    return elevatorRMotor.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    return elevatorRMotor.getVelocity().getValueAsDouble();
  }

  public void setAction(ElevatorAction action) {
    elevatorRMotor.set(action.rate);
  }

  public void setState(ElevatorState state) {
    elevatorRMotor.setControl(new PositionDutyCycle(state.position));
  }

  public void setHold() {
    elevatorRMotor.setControl(new PositionDutyCycle(getCurrentHeight()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", getCurrentHeight());
    SmartDashboard.putNumber("Elevator Velocity", getVelocity());
  }
}
