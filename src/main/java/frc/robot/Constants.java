// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // OIController
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final boolean kDriverFieldOriented = true;
    public static final double kDeadband = 0.1;

    public static double deadbandHandler(double value, double deadband) {
      if (Math.abs(value) < deadband) {
        return 0;
      } else if (value > 0) {
        return (value - OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      } else {
        return (value + OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      }
    }
  }

  // ID
  public static class IDConstants {
    // Swerve Drive Motor Port
    public static final int kFrontLeftDrivePort = 4;
    public static final int kFrontRightDrivePort = 3;
    public static final int kBackLeftDrivePort = 2;// 1
    public static final int kBackRightDrivePort = 1;// 2

    // Swerve Turning Motor Port
    public static final int kFrontLeftTurnPort = 8;
    public static final int kFrontRightTurnPort = 7;
    public static final int kBackLeftTurnPort = 6;// 5
    public static final int kBackRightTurnPort = 5;// 6

    // Elevator Motor Port
    public static final int kElevatorLMotor = 9;
    public static final int kElevatorRMotor = 10;

    // Grabber Angle Motor Port
    public static final int kAlgaeVortex = 11;
    public static final int kAlgaeGrabberAngle = 12;

    // Grabber Motor Port
    public static final int kCoralVortex = 13;
    public static final int kCoralGrabberAngle = 14;

    // Climber Motor Port
    public static final int kClimberLMotor = 15;
    public static final int kClimberRMotor = 16;

  }

  // SwerveModule
  public static class ModuleConstants {
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1.0 / 6.12; // MK3:1.0 / 8.16;
    public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);// MK3: 1 / 8.16
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    // Used in working code currently
    public static final double kPTurning = 0.5;

    // These two used for simulation currently
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.005;

  }

  // SwerveDrive
  public static class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = 0.720;

    // Distance between front and back wheels
    public static final double kWheelBase = kTrackWidth;

    // Need to update to correct values, I dont remember the value we set last meet
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackLeftDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    // -------> ABE <-------- //
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;// 1
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;// 2

    public static final double kPhysicalMaxSpeedMetersPerSecond = 2.5;// 5.5
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;// 2

    public static final double kPTheta = 0.012;// 0.012
    public static final double kITheta = 0.0;// 0.01
    public static final double kDTheta = 0.00015;// 0.00015
    public static final double kIZTheta = 60.0;// 60.0

    public static final double kMaxDriveMotorTemp = 33.0;

    public static final double kMotorMaxOutput = 1;

    public static final double kPLockHeading = 0.02;
    public static final double kILockHeading = 0.025;
    public static final double kDLockHeading = 0.002;

  }

  // Auto
  public static final class AutoConstants {
    public static final double kAutoDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

    public static final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0), // Translation constants
        new PIDConstants(3, 0, 0), // Rotation constants
        // // Drive base radius (distance from center to furthest module)
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm());
  }

  // Elevator
  public static final class ElevatorConstants {

    public static final double kDefault = 0;
    public static final double kL1 = 71.00; //L1
    public static final double kL2 = 137.0; //L2
    public static final double kL3 = 234.0; //L3 & Top
    public static final double kL4 = 215.67; //L4
    // public static final double kTop = 234.00; //L4 & L3

    public static final double kElevatorMotorRate = 1.0;

    public static final double kUpLimit = 234.00;
    public static final double kDefaultLimit = 0.0;

    public static final double kElevatorMotorGearRatio = 48.0 * (22.0 / 22.0);

    // Elevator PID
    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;


    public enum ElevatorState {
      kDefault,
      kL1,
      kL2,
      kL3,
      kL4,
      kUP,
      kDown,
      kStop
    }

  }

  // Coral Grabber
  public static class CoralGrabberConstants {
    public static final double kCoralDefultPosition = 0.985;
    public static final double kL1Position = 0.953; //L1
    public static final double kL2Psoition = 0.953;
    public static final double kL3Position = 0.93;
    public static final double kCoralTopPosition = 0.5;

    public static final double kCoralAngleMotorRate = 0.5;
    public static final double CoralmotorRate = 0.15;
    public static final double CoralmotorFwd = 0.1;

    public static final double kCoralUpLimit = 0.99;
    public static final double kCoralDownLimit = 0.48;

    public static final double kcorlorSensorGateValue = 300;
    // public static final double kcorlorSensorLGateValue = 90;

    public static final double kCoralVortexRatio = 26.0 / 14.0;
    public static final double kCoralAngleMotorRatio = 100.0 * (38.0 / 12.0) * (36.0 / 22.0);

    
    // Coral Angle PID
    public static final double CoralkP = 6.0;
    public static final double CoralkI = 0.005;
    public static final double CoralkD = 2;
    public static final double CoralkIz = 0.1;
    public static final double CoralkFF = 0;
    public static final double CoralkMaxOutput = 1;
    public static final double CoralkMinOutput = -1;

    public enum CoralState {
      kCoralFwd,
      kCoralRev,
      kgetCoral,
      kCoralUP,
      kCoralDown,
      kCoralDefult,
      kL1,
      kL2,
      kL3,
      kCoralTop,
      kCoralStop
    }
  }

  //Algae Grabber
  public static final class AlgaeGrabberConstants {

    public static final double AlgaeMotorRate = 0.3;//0.2
    public static final double kAlgaeAngleMotorRate = 0.5;

    public static final double kAlgaeUpLimit = 0.31;
    public static final double kAlgaeDownLimit = 0.07;

    public static final double kAlgaeDefultPosition = 0.31;
    public static final double kGetL2AlgaePosition = 0.11;
    public static final double kGetL3AlgaePosition = 0.186;
    public static final double kAlgaePutProPosition = 0.07;

    // Algae Angle PID
    public static final double AlgaekP = 10.5;
    public static final double AlgaekI = 0.0;
    public static final double AlgaekD = 0.0;
    public static final double AlgaekIz = 0;
    public static final double AlgaekFF = 0;
    public static final double AlgaekMaxOutput = 1;
    public static final double AlgaekMinOutput = -1;

    public enum AlgaeState {
      kAlgaeUP,
      kAlgaeDown,
      kAlgaeDefult,
      // kAlgaePutPro,
      kAlgaeTop,
      kgetAlgae,
      kputAlgae,
      kAlgaeStop
    }
  }

  // Climber
  public static final class ClimberConstants {
    public static final double ClimberMotorRate = 0.5;

    public static final double ClimberUPLimit = 0;
    public static final double ClimberDownLimit = 0;

  }

  // Status
  public static final class StatusConstants {
    // LED
    public static final int LED_LEFT_START = 0;
    public static final int LED_LEFT_END = 134;
    public static final int LED_RIGHT_START = 135;
    public static final int LED_RIGHT_END = 269;

  }

  // Limelight
  public static final class LimelightConstants {

    // Limelight Name Mapping
    public enum Limelight {
      // kCoral("limelight-b", 1), // IP: 10.81.69.13
      kReef("limelight-c", 1); // IP: 10.81.69.15

      public final String hostname;
      public final double approachingYSpeed;

      private Limelight(String hostname, double approachingYSpeed) {
        this.hostname = hostname;
        this.approachingYSpeed = approachingYSpeed;
      }
    }
  }
}