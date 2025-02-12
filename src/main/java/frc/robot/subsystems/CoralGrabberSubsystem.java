// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig.Type;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralGrabberConstants;
// import frc.robot.Constants.CoralGrabberConstants;
// import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.IDConstants;

public class CoralGrabberSubsystem extends SubsystemBase {
  private SparkMax coralVortex = new SparkMax(IDConstants.kCoralVortex, MotorType.kBrushless);
  private SparkMax CoralGrabberAngleMotor = new SparkMax(IDConstants.kCoralGrabberAngle, MotorType.kBrushless);
  private SparkMaxConfig coralVortexConfig = new SparkMaxConfig();
  private SparkMaxConfig CoralGrabberAngleConfig = new SparkMaxConfig();
  private RelativeEncoder CoralGrabberAngleEncoder = CoralGrabberAngleMotor.getEncoder();
  private SparkAbsoluteEncoder CoralGrabberAngleAbsEncoder = CoralGrabberAngleMotor.getAbsoluteEncoder();
  private SparkClosedLoopController CoralGrabberAnglePIDController = CoralGrabberAngleMotor.getClosedLoopController();

  private DigitalInput irL = new DigitalInput(0);
  private DigitalInput irR = new DigitalInput(1);

  // private I2C.Port i2cPort = I2C.Port.kOnboard;
  // private ColorSensorV3 coralSensorR = new ColorSensorV3(i2cPort);
  // private ColorSensorV3 coralSensorL = new ColorSensorV3(i2cPort);

  /** Creates a new GrabberSubsystem. */
  public CoralGrabberSubsystem() {

    CoralGrabberAngleEncoder.setPosition(CoralGrabberAngleAbsEncoder.getPosition());
    // coralSensorR.configureProximitySensor(ProximitySensorResolution.kProxRes11bit,
    // ProximitySensorMeasurementRate.kProxRate12ms);

    coralVortexConfig
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(70);

    CoralGrabberAngleConfig.softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(CoralGrabberConstants.kCoralUpLimit)
        .reverseSoftLimit(CoralGrabberConstants.kCoralDownLimit);

    CoralGrabberAngleConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(CoralGrabberConstants.CoralkP, CoralGrabberConstants.CoralkI, CoralGrabberConstants.CoralkD)
        .iZone(CoralGrabberConstants.CoralkIz)
        .velocityFF(CoralGrabberConstants.CoralkFF)
        .maxOutput(CoralGrabberConstants.CoralkMaxOutput)
        .minOutput(CoralGrabberConstants.CoralkMinOutput);

    coralVortex.configure(coralVortexConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    CoralGrabberAngleMotor.configure(CoralGrabberAngleConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // coralSensorL.configureProximitySensor(ProximitySensorResolution.kProxRes11bit,
    // ProximitySensorMeasurementRate.kProxRate25ms);

  }

  public boolean getIR(){
    return !irL.get() || !irR.get();
  }

  // return Angle Relativ6e Position
  public double getCoralPosition() {
    return CoralGrabberAngleEncoder.getPosition();
  }

  // return Angle Velocity
  public double getCoralVelocity() {
    return CoralGrabberAngleEncoder.getVelocity();
  }

  // return Angle Absolute Position
  public double getCoralAbsPosition() {
    return CoralGrabberAngleAbsEncoder.getPosition();
  }

  // public double getRIR() {
  // /**
  // * The sensor returns a raw IR value of the infrared light detected.
  // */
  // double RIR = coralSensorR.getIR();
  // return RIR;
  // }

  // public double getLIR() {
  // /**
  // * The sensor returns a raw IR value of the infrared light detected.
  // */
  // double LIR = coralSensorL.getIR();
  // return LIR;
  // }

  // public int getRProximity() {
  // /**
  // * In addition to RGB IR values, the color sensor can also return an
  // * infrared proximity value. The chip contains an IR led which will emit
  // * IR pulses and measure the intensity of the return. When an object is
  // * close the value of the proximity will be large (max 2047 with default
  // * settings) and will approach zero when the object is far away.
  // *
  // * Proximity can be used to roughly approximate the distance of an object
  // * or provide a threshold for when an object is close enough to provide
  // * accurate color values.
  // */
  // int Rproximity = coralSensorR.getProximity();
  // return Rproximity;
  // }

  // public int getLProximity() {
  // /**
  // * In addition to RGB IR values, the color sensor can also return an
  // * infrared proximity value. The chip contains an IR led which will emit
  // * IR pulses and measure the intensity of the return. When an object is
  // * close the value of the proximity will be large (max 2047 with default
  // * settings) and will approach zero when the object is far away.
  // *
  // * Proximity can be used to roughly approximate the distance of an object
  // * or provide a threshold for when an object is close enough to provide
  // * accurate color values.
  // */
  // int Lproximity = coralSensorL.getProximity();
  // return Lproximity;
  // }

  // public boolean RisPass() {
  // return getRProximity() > CoralGrabberConstants.kcorlorSensorGateValue;
  // }

  // public boolean LisPass() {
  // return getLProximity() > CoralGrabberConstants.kcorlorSensorGateValue;
  // }

  public void setDefultPosition() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kCoralDefultPosition, ControlType.kPosition);
  }

  public void setL1Position() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kL1Position, ControlType.kPosition);
  }

  public void setL2Postion() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kL2Psoition, ControlType.kPosition);
  }

  public void setL3Position() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kL3Position, ControlType.kPosition);
  }

  public void setCoralTopPosition() {
    CoralGrabberAnglePIDController.setReference(CoralGrabberConstants.kCoralTopPosition, ControlType.kPosition);
  }

  public void CoralGrabberAngleUP() {
    CoralGrabberAngleMotor.set(CoralGrabberConstants.kCoralAngleMotorRate);
  }

  public void CoralGrabberAngleDown() {
    CoralGrabberAngleMotor.set(-CoralGrabberConstants.kCoralAngleMotorRate);
  }

  public void CoralGrabberAngleStop() {
    CoralGrabberAngleMotor.set(0);
  }

  public void CoralGrabberAngleHold() {
    CoralGrabberAnglePIDController.setReference(getCoralAbsPosition(), ControlType.kPosition);
  }

  public void CoralFwd() {
    coralVortex.set(CoralGrabberConstants.CoralmotorRate);
  }

  public void CoralRev() {
    coralVortex.set(-CoralGrabberConstants.CoralmotorRate);
  }

  public void CoralRevSlow() {
    coralVortex.set(-0.08);
  }

  public void runFwd() {
    coralVortex.set(CoralGrabberConstants.CoralmotorFwd);
  }

  // public void CoralFwdSce() {
  // coralVortex.set(CoralGrabberConstants.CoralmotorFwd);
  // }

  public void StopMotor() {
    coralVortex.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Coral Grabber Position", getCoralPosition());
    SmartDashboard.putNumber("Coral Grabber Abs Position", getCoralAbsPosition());
    SmartDashboard.putNumber("Coral Grabber Velocity", getCoralVelocity());
    // SmartDashboard.putBoolean("isGet", RisPass());

    SmartDashboard.putBoolean("IR L", irL.get());
    SmartDashboard.putBoolean("IR R", irR.get());
    SmartDashboard.putBoolean("IR", getIR());
  }

}
