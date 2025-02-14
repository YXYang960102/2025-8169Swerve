// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberState;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.commands.State.StateAuto;
import frc.robot.commands.Swerve.SwerveAutoGo;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsytem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PutCoralAngleAuto extends ParallelDeadlineGroup {
  /** Creates a new GrabberAuto. */
  public PutCoralAngleAuto(
    SwerveSubsytem swerveSubsytem,
    CoralGrabberSubsystem coralGrabberSubsystem,
    AlgaeGrabberSubsystem algaeGrabberSubsystem,
    ElevatorSubsystem elevatorSubsystem
  ) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new StateAuto(algaeGrabberSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kDefault, CoralGrabberState.kL4, AlgaeGrabberState.kGetL2));
    addCommands(new SwerveAutoGo(swerveSubsytem, Limelight.kReef));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
