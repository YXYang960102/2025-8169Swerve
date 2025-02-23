// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberState;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateAuto extends InstantCommand {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorState elevatorState;
  private CoralGrabberState coralState;
  private AlgaeGrabberState algaeState;

  // private boolean algaeMoved = false;
  // private boolean coralMoved = false;
  // Timer timer = new Timer();

  /** Creates a new AngleAuto. */
  public StateAuto(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElevatorState elevatorState,
      CoralGrabberState coralState,
      AlgaeGrabberState algaeState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorState = elevatorState;
    this.coralState = coralState;
    this.algaeState = algaeState;

    addRequirements(algaeGrabberSubsystem);
    addRequirements(coralGrabberSubsystem);
    addRequirements(elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(elevatorState!=null){
      elevatorSubsystem.setState(elevatorState);
    }

    if(algaeState!=null){
      algaeGrabberSubsystem.setState(algaeState);
    }

    if(coralState!=null && coralGrabberSubsystem.coralBlock != true){
      coralGrabberSubsystem.setState(coralState);

      if(coralState == CoralGrabberState.kL4){
        coralGrabberSubsystem.coralBlock = true;
      }
    }
    
  }
}
