package frc.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.StatusSubsystem;
import frc.robot.subsystems.StatusSubsystem.LEDState;

public class SetLEDStateCommand extends InstantCommand {
  private final StatusSubsystem stateSubsystem;
  private final LEDState targetState;

  public SetLEDStateCommand(StatusSubsystem stateSubsystem, LEDState targetState) {
    this.stateSubsystem = stateSubsystem;
    this.targetState = targetState;
    addRequirements(stateSubsystem);
  }

  @Override
  public void initialize() {
    stateSubsystem.setLEDState(targetState);
  }
}

