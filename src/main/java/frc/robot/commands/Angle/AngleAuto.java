// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeState;
import frc.robot.Constants.CoralGrabberConstants.CoralState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;

import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleAuto extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorState elevatorState;
  private CoralState coralState;
  private AlgaeState algaeState;
  private boolean coralEnd = false;

  // private boolean algaeMoved = false;
  // private boolean coralMoved = false;
  // Timer timer = new Timer();

  /** Creates a new AngleAuto. */
  public AngleAuto(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElevatorState elevatorState,
      CoralState coralState,
      AlgaeState algaeState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorState = elevatorState;
    this.coralState = coralState;
    this.algaeState = algaeState;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevatorState == ElevatorState.kL1 && algaeState == AlgaeState.kAlgaeTop) { // L1
      elevatorSubsystem.setL1();
      algaeGrabberSubsystem.setAlgaeL2Position();
      if (coralState == CoralState.kL1) {
        coralGrabberSubsystem.setL1Position();
      }
    }
    else if (elevatorState == ElevatorState.kL2 && algaeState == AlgaeState.kAlgaeTop) { // L2
      elevatorSubsystem.setL2();
      algaeGrabberSubsystem.setAlgaeL2Position();
      if (coralState == CoralState.kL2) {
        coralGrabberSubsystem.setL1Position();
      }
    }

    else if (elevatorState == ElevatorState.kL3 && algaeState == AlgaeState.kAlgaeTop) { // L3
      elevatorSubsystem.setL3();
      algaeGrabberSubsystem.setAlgaeL3Position();
      if (coralState == CoralState.kL3) {
        coralGrabberSubsystem.setL3Position();
        // if(!coralEnd){
        //   algaeGrabberSubsystem.setDefultPosition();
        //   coralEnd = true;
        // }
      }
    }
    else if (elevatorState == ElevatorState.kL4 && algaeState == AlgaeState.kAlgaeTop) { // L4
      elevatorSubsystem.setL4();
      algaeGrabberSubsystem.setAlgaeL2Position();
      if (coralState == CoralState.kCoralTop) {
        coralGrabberSubsystem.setCoralTopPosition();
        // if(!coralEnd){
        //   algaeGrabberSubsystem.setDefultPosition();
        //   coralEnd = true;
        // }
      }
    }
    else if (elevatorState == ElevatorState.kDefault && algaeState == AlgaeState.kAlgaeTop) { // processor
      elevatorSubsystem.setDefault();
      algaeGrabberSubsystem.setAlgaePutProPosition();
      if (coralState == CoralState.kCoralTop) {
        coralGrabberSubsystem.setCoralTopPosition();
      }
    }
    else if(coralState == CoralState.kCoralTop){
      coralGrabberSubsystem.setCoralTopPosition();
      // algaeGrabberSubsystem.setDefultPosition();
    } 
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
