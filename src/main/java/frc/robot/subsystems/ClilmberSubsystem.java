// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ClimberConstants;
// import frc.robot.Constants.IDConstants;

// public class ClilmberSubsystem extends SubsystemBase {
//   private SparkMax climberLMotor = new SparkMax(IDConstants.kClimberLMotor, MotorType.kBrushless);
//   private SparkMax climberRMotor = new SparkMax(IDConstants.kClimberRMotor, MotorType.kBrushless);
//   private SparkMaxConfig climberLConfig = new SparkMaxConfig();
//   private SparkMaxConfig climberRConfig = new SparkMaxConfig();
//   private RelativeEncoder climberEncoder = climberRMotor.getEncoder();
//   /** Creates a new ClilmberSubsystem. */
//   public ClilmberSubsystem() {
//     climberLConfig
//     .idleMode(IdleMode.kBrake)
//     .inverted(false);

//     climberRConfig
//     .idleMode(IdleMode.kBrake)
//     .inverted(false)
//     .smartCurrentLimit(70);
//   }

//   public double getPosition() {
//     return climberEncoder.getPosition();
//   }

//   public double getVelocity() {
//     return climberEncoder.getVelocity();
//   }

//   public void ClimberDown() {
//     climberRMotor.set(ClimberConstants.ClimberMotorRate);
//     climberLConfig.follow(climberRMotor, false);
//   }

//   public void ClimberUP() {
//     climberRMotor.set(-ClimberConstants.ClimberMotorRate);
//     climberLConfig.follow(climberRMotor, false);
//   }

//   public void ClimberStop() {
//     climberRMotor.set(0);
//     climberLConfig.follow(climberRMotor, false);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("Climber Position", getPosition());
//     SmartDashboard.putNumber("Climber Velocity", getVelocity());
//   }
// }
