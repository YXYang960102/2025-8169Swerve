// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeState;
import frc.robot.Constants.CoralGrabberConstants.CoralState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleAutoDefult extends Command {
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorState elevatorState;
  private CoralState coralState;
  private AlgaeState algaeState;

  private boolean coralMoved = false;
  Timer timer = new Timer();

  /** Creates a new AngleAutoDefult. */
  public AngleAutoDefult(
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      CoralGrabberSubsystem coralGrabberSubsystem,
      ElevatorSubsystem elevatorSubsystem,
      ElevatorState elevatorState,
      CoralState coralState,
      AlgaeState algaeState
  ) {
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
   
   if(elevatorState == ElevatorState.kDefault && coralState == CoralState.kCoralDefult){
    elevatorSubsystem.setDefault();
    coralGrabberSubsystem.setDefultPosition();
    timer.reset();
    timer.start();
    coralMoved = false;
   }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!coralMoved && algaeState == AlgaeState.kAlgaeDefult && timer.get() > 2) {
      coralMoved = true;
      algaeGrabberSubsystem.setDefultPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !coralMoved && timer.get() >= 2.0;
  }
}
