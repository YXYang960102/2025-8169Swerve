// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAction;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeGrabberNormal extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private AlgaeGrabberAction algaeGrabberAction;

  /** Creates a new GrabberNormal. */
  public AlgaeGrabberNormal(
    AlgaeGrabberSubsystem algaeGrabberSubsystem,
    AlgaeGrabberAction algaeGrabberAction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.algaeGrabberAction = algaeGrabberAction;
    addRequirements(algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeGrabberSubsystem.setGrabberAction(algaeGrabberAction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeGrabberSubsystem.setGrabberAction(AlgaeGrabberAction.kStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
