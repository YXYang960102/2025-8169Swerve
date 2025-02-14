// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAuto extends InstantCommand {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ElevatorState elevatorState;
  
  public ElevatorAuto(ElevatorSubsystem elevatorSubsystem, ElevatorState elevatorState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorState = elevatorState;

    addRequirements(elevatorSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setState(elevatorState);;
  }
}
