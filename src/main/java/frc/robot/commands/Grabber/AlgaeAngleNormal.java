// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAngleAction;
import frc.robot.subsystems.AlgaeGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAngleNormal extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private AlgaeGrabberAngleAction algaeGrabberAngleAction;

  /** Creates a new GrabberNormal. */
  public AlgaeAngleNormal(
    AlgaeGrabberSubsystem algaeGrabberSubsystem,
    AlgaeGrabberAngleAction algaeGrabberAngleAction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.algaeGrabberAngleAction = algaeGrabberAngleAction;
    addRequirements(algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeGrabberSubsystem.setAngleAction(algaeGrabberAngleAction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeGrabberSubsystem.setAngleAction(AlgaeGrabberAngleAction.kStop);
    algaeGrabberSubsystem.setAngleHold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
