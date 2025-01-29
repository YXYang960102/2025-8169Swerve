// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Elevator;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ElevatorConstants.ElevatorState;
// // import frc.robot.Constants.ElevatorConstants.HeightSet;
// import frc.robot.subsystems.ElevatorSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ElevatorAuto extends Command {
//   private final ElevatorSubsystem elevatorSubsystem;
//   private final ElevatorState elevatorState;
//   /** Creates a new ElevatorAuto. */
//   public ElevatorAuto(ElevatorSubsystem elevatorSubsystem, ElevatorState elevatorState) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.elevatorSubsystem = elevatorSubsystem;
//     this.elevatorState = elevatorState;

//     addRequirements(elevatorSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if(elevatorState == ElevatorState.kDefault)
//        elevatorSubsystem.setDefault();
//     if(elevatorState == ElevatorState.kProcessor)
//        elevatorSubsystem.setProcessor();
//     if(elevatorState == ElevatorState.kAlgae)
//        elevatorSubsystem.setAlgae();
//     if(elevatorState == ElevatorState.kReef)
//        elevatorSubsystem.setReef();
//     if(elevatorState == ElevatorState.kCoralStation)
//        elevatorSubsystem.setCoralStation();
//     if(elevatorState == ElevatorState.kTop)
//        elevatorSubsystem.setTop();
  
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     elevatorSubsystem.ElevatorStop(); 
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }
