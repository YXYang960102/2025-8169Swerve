// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.GrabberConstants;
// import frc.robot.Constants.IDConstants;

// public class GrabberSubsystem extends SubsystemBase {
//   private SparkMax coralVortex = new SparkMax(IDConstants.kCoralVortex, MotorType.kBrushless);
//   private SparkMax algaeLMotor = new SparkMax(IDConstants.kAlgaeLMotor, MotorType.kBrushless);
//   private SparkMax algaeRMotor = new SparkMax(IDConstants.kAlgaeRMotor, MotorType.kBrushless);
//   private SparkMaxConfig coralVortexConfig = new SparkMaxConfig();
//   private SparkMaxConfig algaeLConfig = new SparkMaxConfig();
//   private SparkMaxConfig algaeRConfig = new SparkMaxConfig();
  
//   /** Creates a new GrabberSubsystem. */
//   public GrabberSubsystem() {
//     coralVortexConfig
//     .idleMode(IdleMode.kCoast)
//     .inverted(false)
//     .smartCurrentLimit(70);
    
//     algaeLConfig
//     .idleMode(IdleMode.kBrake)
//     .inverted(false);

//     algaeRConfig
//     .idleMode(IdleMode.kBrake)
//     .inverted(false)
//     .smartCurrentLimit(70);
//   }

//   public void getCoral() {
//     coralVortex.set(GrabberConstants.CoralmotorRate);
//   }

//   public void putCoral() {
//     coralVortex.set(-GrabberConstants.CoralmotorRate);
//   }

//   public void getAlgae() {
//     algaeRMotor.set(GrabberConstants.AlgaeMotorRate);
//     algaeLConfig.follow(algaeRMotor, false);
//   }

//   public void putAlgae() {
//     algaeRMotor.set(-GrabberConstants.AlgaeMotorRate);
//     algaeLConfig.follow(algaeRMotor, false);
//   }

//   public void StopAllMotor() {
//     algaeLMotor.set(0);
//     algaeRMotor.set(0);
//     coralVortex.set(0);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

  
// }
