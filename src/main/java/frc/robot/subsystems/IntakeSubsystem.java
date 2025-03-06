// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants.IntakeAction;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private SparkMax IntakeMotor = new SparkMax(IDConstants.kIntakeMotor, MotorType.kBrushless);
private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    IntakeMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(70);

    IntakeMotor.configure(IntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public double getIntakeVolage() {
    return IntakeMotor.getOutputCurrent();
  }

  public void setIntakeAction(IntakeAction action) {
    IntakeMotor.set(action.rate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
