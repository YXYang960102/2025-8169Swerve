// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveLockHeading extends Command {
  private final SwerveSubsytem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  protected final Limelight limelight;
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  PIDController pidController = new PIDController(
    DriveConstants.kPLockHeading, 
    DriveConstants.kILockHeading, 
    DriveConstants.kDLockHeading);

  /** Creates a new SwerveLockHeading. */
  public SwerveLockHeading(
      SwerveSubsytem swerveSubsystem,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.limelight = limelight;

    // Slew rate limiter
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Set joystick inputs to speed variables
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningAngle = turningSpdFunction.get();

    // Apply deadband to protect motors
    xSpeed = OIConstants.deadbandHandler(xSpeed, OIConstants.kDeadband);
    ySpeed = OIConstants.deadbandHandler(ySpeed, OIConstants.kDeadband);
    turningAngle = Math.abs(turningAngle) > OIConstants.kDeadband ? turningAngle / (1 - OIConstants.kDeadband) : 0.0;

    // Apply slew rate to joystick input to make robot input smoother and mulitply
    // by max speed
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    turningAngle = turningLimiter.calculate(turningAngle);

    if (turningAngle == 0 && LimelightHelpers.getTV(limelight.hostname)) {

    // double targetPose[] = LimelightHelpers.getTargetPose_RobotSpace(limelight.hostname);
    // double distanceToTag = targetPose.length >= 3 ? targetPose[0] : 1.0;
    // double angleCompensation = Math.toDegrees(Math.atan(0.1 / distanceToTag));
    // turningAngle = pidController.calculate(LimelightHelpers.getTX(limelight.hostname) + angleCompensation, 0);

      turningAngle = pidController.calculate(LimelightHelpers.getTX(limelight.hostname), 0);
      swerveSubsystem.setChassisOutput(xSpeed, ySpeed, turningAngle, true);
    } else {
      turningAngle *= 3;
      swerveSubsystem.setChassisOutput(xSpeed, ySpeed, turningAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
