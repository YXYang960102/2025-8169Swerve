// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGrabberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralAuto extends Command {
  private CoralGrabberSubsystem coralGrabberSubsystem;
  private boolean running = false;
  Timer timer = new Timer();

  /** Creates a new CoralAuto. */
  public CoralAuto(CoralGrabberSubsystem coralGrabberSubsystem) {
    this.coralGrabberSubsystem = coralGrabberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // coralGrabberSubsystem.CoralRevSlow();
    timer.reset();
    running = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!running && timer.get() == 0){
      timer.start();
      coralGrabberSubsystem.CoralRevSlow();
      running = true;
      
    }
    // if(!running && coralGrabberSubsystem.getIR()){
    //   coralGrabberSubsystem.CoralRevSlow();
    //   running = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralGrabberSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.0;// running && !coralGrabberSubsystem.getIR();
  }
}
