// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Grabber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AngleConstants.AngleState;
// import frc.robot.subsystems.GrabberAngleSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AngleNoraml extends Command {
//   private GrabberAngleSubsystem grabberAngleSubsystem;
//   private AngleState angleState;

//   /** Creates a new AngleNoraml. */
//   public AngleNoraml(
//       GrabberAngleSubsystem grabberAngleSubsystem,
//       AngleState angleState) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.grabberAngleSubsystem = grabberAngleSubsystem;
//     this.angleState = angleState;

//     addRequirements(grabberAngleSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (angleState == AngleState.kUP)
//       grabberAngleSubsystem.AngleUP();
//     if (angleState == AngleState.kDown)
//       grabberAngleSubsystem.AngleDown();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     grabberAngleSubsystem.AngleStop();
//     grabberAngleSubsystem.AngleHold();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
