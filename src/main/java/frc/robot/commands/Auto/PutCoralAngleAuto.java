// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeState;
import frc.robot.Constants.CoralGrabberConstants.CoralState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.commands.Angle.AngleAuto;
import frc.robot.commands.Grabber.GrabberNormal;
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
    super(new AngleAuto(algaeGrabberSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kDefault, CoralState.kCoralTop, AlgaeState.kAlgaeTop));
    addCommands(new SwerveAutoGo(swerveSubsytem, Limelight.kReef));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
