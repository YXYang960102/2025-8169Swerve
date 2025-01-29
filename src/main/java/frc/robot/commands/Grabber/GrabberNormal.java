// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Grabber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.GrabberConstants.GrabberMode;
// import frc.robot.subsystems.GrabberSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class GrabberNormal extends Command {
//   private GrabberSubsystem grabberSubsystem;
//   private GrabberMode grabberMode;

//   /** Creates a new GrabberNormal. */
//   public GrabberNormal(
//       GrabberSubsystem grabberSubsystem,
//       GrabberMode grabberMode) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.grabberSubsystem = grabberSubsystem;
//     this.grabberMode = grabberMode;

//     addRequirements(grabberSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (grabberMode == GrabberMode.kgetCoral)
//       grabberSubsystem.getCoral();
//     if (grabberMode == GrabberMode.kputCoral)
//       grabberSubsystem.putCoral();
//     if (grabberMode == GrabberMode.kgetAlgae)
//       grabberSubsystem.getAlgae();
//     if (grabberMode == GrabberMode.kputAlgae)
//       grabberSubsystem.putAlgae();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     grabberSubsystem.StopAllMotor();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
