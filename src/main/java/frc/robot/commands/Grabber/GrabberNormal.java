// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GrabberConstants.GrabberMode;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberNormal extends Command {
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private AlgaeGrabberSubsystem algaeGrabberSubsystem;
  private GrabberMode grabberMode;

  /** Creates a new GrabberNormal. */
  public GrabberNormal(
      CoralGrabberSubsystem coralGrabberSubsystem,
      AlgaeGrabberSubsystem algaeGrabberSubsystem,
      GrabberMode grabberMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coralGrabberSubsystem = coralGrabberSubsystem;
    this.algaeGrabberSubsystem = algaeGrabberSubsystem;
    this.grabberMode = grabberMode;

    // addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (grabberMode == GrabberMode.kgetCoral)
      coralGrabberSubsystem.getCoral();
    if (grabberMode == GrabberMode.kputCoral)
      coralGrabberSubsystem.putCoral();
    if (grabberMode == GrabberMode.kgetAlgae)
      algaeGrabberSubsystem.getAlgae();
    if (grabberMode == GrabberMode.kputAlgae)
      algaeGrabberSubsystem.putAlgae();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralGrabberSubsystem.StopMotor();
    algaeGrabberSubsystem.AlgaeMotorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
