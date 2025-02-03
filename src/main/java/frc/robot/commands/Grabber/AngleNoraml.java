// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants.AngleState;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleNoraml extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private AngleState angleState;

  /** Creates a new AngleNoraml. */
  public AngleNoraml(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      AngleState angleState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.angleState = angleState;

    addRequirements(algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (angleState == AngleState.kCoralUP)
      coralGrabberSubsystem.CoralGrabberAngleUP();
    if (angleState == AngleState.kCoralDown)
      coralGrabberSubsystem.CoralGrabberAngleDown();
    if (angleState == AngleState.kAlgaeUP)
      algaeGrabberSubsystem.AlgaeGrabberAngleUP();
    if(angleState == AngleState.kAlgaeDown)
      algaeGrabberSubsystem.AlgaeGrabberAngleDown();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralGrabberSubsystem.CoralGrabberAngleStop();
    algaeGrabberSubsystem.AlgaeAngleMotorStop();
    coralGrabberSubsystem.CoralGrabberAngleHold();
    algaeGrabberSubsystem.AlgaeGrabberAngleHold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
