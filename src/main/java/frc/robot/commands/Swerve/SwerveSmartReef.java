// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveSmartReef extends Command {
  private SwerveSubsytem swerveSubsystem;
  private final Limelight limelight;
  private final PIDController pidController = new PIDController(0.006, 0.005, 0);
  private final DoubleSupplier speedSup;

  /** Creates a new SwerveSmartReef. */
  public SwerveSmartReef(SwerveSubsytem swerveSubsytem, Limelight limelight, DoubleSupplier speedSup) {
    this.swerveSubsystem = swerveSubsytem;
    this.limelight = limelight;
    this.speedSup = speedSup;
    // Use addRequirements() here to declare subsystem dependencies.
    pidController.setIZone(5);

    addRequirements(swerveSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = OIConstants.deadbandHandler(speedSup != null ? speedSup.getAsDouble() : 0, 0.4) * 0.2;
    if (LimelightHelpers.getTV(limelight.hostname)) {
      swerveSubsystem.setChassisOutput(speed * limelight.approachingXSpeed,
          pidController.calculate(LimelightHelpers.getTX(limelight.hostname), 0),
          AprilTagConstants.ID2Angle[(int) LimelightHelpers.getFiducialID(limelight.hostname) - 1], true, true);
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
    return false;
  }
}
