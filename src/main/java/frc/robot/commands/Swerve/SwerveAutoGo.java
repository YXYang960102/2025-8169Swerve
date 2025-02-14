// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.LimelightHelpers;

import frc.robot.subsystems.SwerveSubsytem;

public class SwerveAutoGo extends Command {
  private final SwerveSubsytem swerveSubsystem;
  private final Limelight limelight;
  private final DoubleSupplier speed;
  private boolean detected;

  PIDController pidController = new PIDController(
    DriveConstants.kPLockHeading, 
    DriveConstants.kILockHeading, 
    DriveConstants.kDLockHeading);

  /** Creates a new SwerveAutoGo. */
  public SwerveAutoGo(SwerveSubsytem swerveSubsystem, Limelight limelight) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.speed = null;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  public SwerveAutoGo(SwerveSubsytem swerveSubsystem, Limelight limelight, DoubleSupplier speed) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    detected = false;
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getTV(limelight.hostname)) {
      detected = true;

      double turningAngle = pidController.calculate(LimelightHelpers.getTX(limelight.hostname), 0);
      double xSpeed;
      if (speed != null) {
        xSpeed = OIConstants.deadbandHandler(speed.getAsDouble(), 0.1) * 0.5;
      } else {
        xSpeed = LimelightHelpers.getTY(limelight.hostname) * 0.01;
      }
      swerveSubsystem.setChassisOutput(xSpeed * limelight.approachingXSpeed, 0, turningAngle, true, true);
    } else {
      swerveSubsystem.stopModules();
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
    return speed == null && detected && !LimelightHelpers.getTV(limelight.hostname);
  }
}