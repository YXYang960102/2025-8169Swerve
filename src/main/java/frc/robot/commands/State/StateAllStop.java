// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.State;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAngleAction;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberAngleAction;
import frc.robot.Constants.ElevatorConstants.ElevatorAction;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateAllStop extends InstantCommand {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  /** Creates a new AngleAllStop. */
  public StateAllStop(
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
    elevatorSubsystem.setAction(ElevatorAction.kStop);
    coralGrabberSubsystem.setAngleAction(CoralGrabberAngleAction.kStop);
    algaeGrabberSubsystem.setAngleAction(AlgaeGrabberAngleAction.kStop);
  }

}
