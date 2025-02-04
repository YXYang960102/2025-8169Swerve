// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.Constants.AngleConstants.AngleState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.GrabberConstants.GrabberMode;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleAuto extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorState elevatorState;
  private AngleState angleState;

  /** Creates a new AngleAuto. */
  public AngleAuto(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElevatorState elevatorState,
      AngleState angleState) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorState = elevatorState;
    this.angleState = angleState;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevatorState == ElevatorState.kDefault && angleState == AngleState.kCoralDefult && angleState == AngleState.kAlgaeDefult) {
      coralGrabberSubsystem.setDefultPosition();
      algaeGrabberSubsystem.setDefultPosition();
      elevatorSubsystem.setDefault();
    }
    if (elevatorState == ElevatorState.kL2 && angleState == AngleState.kCoralDefult && angleState == AngleState.kAlgaeTop) { //L2
      coralGrabberSubsystem.setDefultPosition();
      algaeGrabberSubsystem.setAlgaeTopPosition();
      elevatorSubsystem.setL2();
    }
    if (elevatorState == ElevatorState.kDefault && angleState == AngleState.kCoralTop) { //L3
      coralGrabberSubsystem.setCoralTopPosition();
      algaeGrabberSubsystem.setAlgaeTopPosition();
      elevatorSubsystem.setDefault();
    }
    if (elevatorState == ElevatorState.kCoralStation && angleState == AngleState.kCoralStation) { //Get Coral
      coralGrabberSubsystem.setDefultPosition();
      elevatorSubsystem.setDefault();
    }
    if (elevatorState == ElevatorState.kL4 && angleState == AngleState.kCoralTop) { // L4
      coralGrabberSubsystem.setCoralTopPosition();
      algaeGrabberSubsystem.setAlgaeTopPosition();
      elevatorSubsystem.setL4();
    }
    if (elevatorState == ElevatorState.kTop && angleState == AngleState.kAlgaeTop) { // Top
      coralGrabberSubsystem.setCoralTopPosition();
      algaeGrabberSubsystem.setAlgaeTopPosition();
      elevatorSubsystem.setTop();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeGrabberSubsystem.setDefultPosition();
    coralGrabberSubsystem.setDefultPosition();
    elevatorSubsystem.setDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
