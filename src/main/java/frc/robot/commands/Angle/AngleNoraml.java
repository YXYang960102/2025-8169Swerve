// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeState;
import frc.robot.Constants.CoralGrabberConstants.CoralState;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleNoraml extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private CoralState coralState;
  private AlgaeState algaeState;

  /** Creates a new AngleNoraml. */
  public AngleNoraml(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      CoralState coralState,
      AlgaeState algaeState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.coralState = coralState;
    this.algaeState = algaeState;

    // addRequirements(algaeGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (coralState == CoralState.kCoralUP)
      coralGrabberSubsystem.CoralGrabberAngleUP();
    if (coralState == CoralState.kCoralDown)
      coralGrabberSubsystem.CoralGrabberAngleDown();
    if (algaeState == AlgaeState.kAlgaeUP)
      algaeGrabberSubsystem.AlgaeGrabberAngleUP();
    if (algaeState == AlgaeState.kAlgaeDown)
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
