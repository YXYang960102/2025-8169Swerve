// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.State;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberState;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateAutoDefault extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  /** Creates a new AngleAutoDefult. */
  public StateAutoDefault(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      ElevatorSubsystem elevatorSubsystem
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //To-Do: seq
    if(!algaeGrabberSubsystem.isSafe()){
      algaeGrabberSubsystem.setState(AlgaeGrabberState.kSafe);
    }

    coralGrabberSubsystem.coralBlock = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(algaeGrabberSubsystem.isSafe()){
      elevatorSubsystem.setState(ElevatorState.kDefault);
      coralGrabberSubsystem.setState(CoralGrabberState.kDefult);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted)
      algaeGrabberSubsystem.setState(AlgaeGrabberState.kDefult);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralGrabberSubsystem.isDefault();
  }
}