// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
public class AutoL4 extends ParallelCommandGroup {
  /** Creates a new AutoL4. */
  public AutoL4(
    SwerveSubsytem swerveSubsytem,
    ElevatorSubsystem elevatorSubsystem,
    CoralGrabberSubsystem coralGrabberSubsystem,
    AlgaeGrabberSubsystem algaeGrabberSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SwerveAutoGo(swerveSubsytem, Limelight.kReef, () -> 0.3, true),
      new StateAuto(algaeGrabberSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL4, CoralGrabberState.kL4, AlgaeGrabberState.kGetL2)
    );
  }
}
