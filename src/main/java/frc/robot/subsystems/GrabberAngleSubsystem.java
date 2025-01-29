// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import frc.robot.Constants.AngleConstants;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IDConstants;
// import frc.robot.Constants.AngleConstants.AngleState;

// public class GrabberAngleSubsystem extends SubsystemBase {
//   private SparkMax GrabberAngleMotor = new SparkMax(IDConstants.kGrabberAngle, MotorType.kBrushless);
//   private SparkMaxConfig angleConfig = new SparkMaxConfig();
//   private RelativeEncoder AngleEncoder = GrabberAngleMotor.getEncoder();
//   private SparkAbsoluteEncoder AngleAbsEncoder = GrabberAngleMotor.getAbsoluteEncoder();
//   private SparkClosedLoopController AnglePIDController = GrabberAngleMotor.getClosedLoopController();
//   /** Creates a new GrabberAngleSubsystem. */
//   public GrabberAngleSubsystem() {

//     AngleEncoder.setPosition(AngleAbsEncoder.getPosition());

//     GrabberAngleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     angleConfig
//     .inverted(false)
//     .idleMode(IdleMode.kBrake)
//     .closedLoop
//     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//     .pid(AngleConstants.kP, AngleConstants.kI, AngleConstants.kD)
//     .iZone(AngleConstants.kIz)
//     .velocityFF(AngleConstants.kFF)
//     .maxOutput(AngleConstants.kMaxOutput)
//     .minOutput(AngleConstants.kMinOutput);
    
//     angleConfig.softLimit
//     .forwardSoftLimitEnabled(false)
//     .reverseSoftLimitEnabled(false)
//     .forwardSoftLimit(AngleConstants.kUpLimit)
//     .reverseSoftLimit(AngleConstants.kDownLimit);

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   //return Angle Relative Position
//   public double getPosition() {
//     return AngleEncoder.getPosition();
//   }

//   //return Angle Velocity
//   public double getVelocity() {
//     return AngleEncoder.getVelocity();
//   }

//   //return Angle Absolute Position
//   public double getAbsPosition() {
//     return AngleAbsEncoder.getPosition();
//   }

//   public void setDefultPosition() {
//     AnglePIDController.setReference(AngleConstants.kDefultPosition, ControlType.kPosition);
//   }

//   public void setProcessorPosition() {
//     AnglePIDController.setReference(AngleConstants.kProcessorPosition, ControlType.kPosition);
//   }

//   public void setReefPosition() {
//     AnglePIDController.setReference(AngleConstants.kReefPosition, ControlType.kPosition);
//   }

//   public void setCoralStationPosition() {
//     AnglePIDController.setReference(AngleConstants.kCoralStationPosition, ControlType.kPosition);
//   }

//   public void setTopPosition() {
//     AnglePIDController.setReference(AngleConstants.kTopPosition, ControlType.kPosition);
//   }

//   public void AngleUP() {
//     GrabberAngleMotor.set(AngleConstants.kAngleMotorRate);
//   }

//   public void AngleDown() {
//     GrabberAngleMotor.set(-AngleConstants.kAngleMotorRate);
//   }

//   public void AngleStop() {
//     GrabberAngleMotor.set(0);
//   }

//   public void AngleHold() {
//     AnglePIDController.setReference(getAbsPosition(), ControlType.kPosition);
//   }

// }
