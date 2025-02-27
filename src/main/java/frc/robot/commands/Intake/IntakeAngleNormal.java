// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeAngleAction;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAngleNormal extends Command {
  private IntakeSubsystem intakeSubsystem;
  private IntakeAngleAction intakeAngleAction;
  /** Creates a new IntakeAngleNormal. */
  public IntakeAngleNormal(
    IntakeSubsystem intakeSubsystem,
    IntakeAngleAction intakeAngleAction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.intakeAngleAction = intakeAngleAction;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setAngleAction(intakeAngleAction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setAngleAction(IntakeAngleAction.kStop);
    intakeSubsystem.setAngleHold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
