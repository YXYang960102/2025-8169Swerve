// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Grabber;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AlgaeGrabberConstants.AlgaeState;
// import frc.robot.Constants.CoralGrabberConstants.CoralState;
// import frc.robot.subsystems.AlgaeGrabberSubsystem;
// import frc.robot.subsystems.CoralGrabberSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CoralGrabberAuto extends Command {
//   private CoralGrabberSubsystem coralGrabberSubsystem;
//   // private AlgaeGrabberSubsystem algaeGrabberSubsystem;
//   // private CoralState coralState;
//   // private AlgaeState algaeState;
//   Timer timer = new Timer();
//   private boolean getCoral = false;
//   /** Creates a new GrabberAuto. */
//   public CoralGrabberAuto(
//   CoralGrabberSubsystem coralGrabberSubsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.coralGrabberSubsystem = coralGrabberSubsystem;
//     // this.algaeGrabberSubsystem = algaeGrabberSubsystem;
//     // this.coralState = coralState;
//     // this.algaeState = algaeState;
//     addRequirements(coralGrabberSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     coralGrabberSubsystem.runFwd();
//     timer.reset();
//     getCoral = false;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(!getCoral && coralGrabberSubsystem.RisPass() && timer.get() == 0.0){
//       timer.start();
//       getCoral = true;
//       coralGrabberSubsystem.StopMotor();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//     coralGrabberSubsystem.StopMotor();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return timer.get() > 0.15 ;
//   }
// }
