// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralGrabberConstants.CoralState;
import frc.robot.subsystems.CoralGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralGrabberAuto extends Command {
  CoralGrabberSubsystem coralGrabberSubsystem;
  CoralState coralState;
  /** Creates a new CoralGrabberAuto. */
  public CoralGrabberAuto(
    CoralGrabberSubsystem coralGrabberSubsystem,
    CoralState coralState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.coralState = coralState;

    addRequirements(coralGrabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(coralState == CoralState.kCoralDefult)
     coralGrabberSubsystem.setDefultPosition();
    if(coralState == CoralState.kL1)
     coralGrabberSubsystem.setL1Position();
    // if(coralState == CoralState.kL2)
    //  coralGrabberSubsystem.setL2Postion();
    if(coralState == CoralState.kL3)
    coralGrabberSubsystem.setL3Position();
    if(coralState == CoralState.kCoralTop)
    coralGrabberSubsystem.setCoralTopPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
