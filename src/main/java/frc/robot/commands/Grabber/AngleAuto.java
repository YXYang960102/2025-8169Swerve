// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Grabber;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.GrabberAngleSubsystem;
// import frc.robot.Constants.AngleConstants.AngleState;
// import frc.robot.Constants.ElevatorConstants.ElevatorState;
// import frc.robot.Constants.GrabberConstants.GrabberMode;
// import frc.robot.subsystems.ElevatorSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class AngleAuto extends Command {
//   private GrabberAngleSubsystem grabberAngleSubsystem;
//   private ElevatorSubsystem elevatorSubsystem;
//   private ElevatorState elevatorState;
//   private AngleState angleState;

//   /** Creates a new AngleAuto. */
//   public AngleAuto(
//       GrabberAngleSubsystem grabberAngleSubsystem,
//       ElevatorSubsystem elevatorSubsystem,
//       ElevatorState elevatorState,
//       AngleState angleState) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.grabberAngleSubsystem = grabberAngleSubsystem;
//     this.elevatorSubsystem = elevatorSubsystem;
//     this.elevatorState = elevatorState;
//     this.angleState = angleState;

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if (elevatorState == ElevatorState.kAlgae && angleState == AngleState.kReef) {
//       grabberAngleSubsystem.setReefPosition();
//       elevatorSubsystem.setAlgae();
//     }
//     if (elevatorState == ElevatorState.kReef && angleState == AngleState.kReef) {
//       grabberAngleSubsystem.setReefPosition();
//       elevatorSubsystem.setReef();
//     }
//     if (elevatorState == ElevatorState.kProcessor && angleState == AngleState.kProcessor) {
//       grabberAngleSubsystem.setProcessorPosition();
//       elevatorSubsystem.setProcessor();
//     }
//     if (elevatorState == ElevatorState.kCoralStation && angleState == AngleState.kCoralStation) {
//       grabberAngleSubsystem.setCoralStationPosition();
//       elevatorSubsystem.setCoralStation();
//     }
//     if (elevatorState == ElevatorState.kTop && angleState == AngleState.kTop) {
//       grabberAngleSubsystem.setTopPosition();
//       elevatorSubsystem.setTop();
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     grabberAngleSubsystem.setDefultPosition();
//     elevatorSubsystem.setDefault();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
