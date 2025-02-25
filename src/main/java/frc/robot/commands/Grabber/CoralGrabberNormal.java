// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberAction;
import frc.robot.subsystems.CoralGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralGrabberNormal extends Command {
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private CoralGrabberAction coralGrabberAction;

  /** Creates a new GrabberNormal. */
  public CoralGrabberNormal(
      CoralGrabberSubsystem coralGrabberSubsystem,
      CoralGrabberAction coralGrabberAction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.coralGrabberAction = coralGrabberAction;
    addRequirements(coralGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralGrabberSubsystem.setGrabberAction(coralGrabberAction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralGrabberSubsystem.setGrabberAction(CoralGrabberAction.kStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
