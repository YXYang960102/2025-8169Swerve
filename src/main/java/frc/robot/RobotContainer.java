// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeState;
import frc.robot.Constants.CoralGrabberConstants.CoralState;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.commands.Angle.AlgaeGrabberAuto;
import frc.robot.commands.Angle.AngleAuto;
import frc.robot.commands.Angle.AngleAutoDefult;
import frc.robot.commands.Angle.AngleNoraml;
import frc.robot.commands.Angle.CoralGrabberAuto;
import frc.robot.commands.Auto.PutCoralAngleAuto;
import frc.robot.commands.Elevator.ElevatorAuto;
import frc.robot.commands.Elevator.ElevatorNormal;
import frc.robot.commands.Grabber.CoralAuto;
// import frc.robot.commands.Grabber.CoralGrabberAuto;
import frc.robot.commands.Grabber.GrabberNormal;
import frc.robot.commands.Grabber.GrabberStop;
import frc.robot.commands.Swerve.SwerveAutoGo;
import frc.robot.commands.Swerve.SwerveLockHeading;
// import frc.robot.commands.Swerve.SwerveXMode;
import frc.robot.commands.Swerve.SwerveFieldRelative;
// import frc.robot.subsystems.StatusSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeGrabberSubsystem;
import frc.robot.subsystems.CoralGrabberSubsystem;
import frc.robot.subsystems.SwerveSubsytem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsytem swerveSubsytem = new SwerveSubsytem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final AlgaeGrabberSubsystem algaeGrabberAngleSubsystem = new AlgaeGrabberSubsystem();
  private final CoralGrabberSubsystem coralGrabberSubsystem = new CoralGrabberSubsystem();

  // private final StatusSubsystem statusSubsystem = new StatusSubsystem(9, 270);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private static CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  

  // Create auto chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    // SmartDashboard.putData(elevatorSubsystem);
    
    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
    setDefaultCommand();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.start().whileTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));
    // //LED Status
    // m_operatorController.leftBumper().whileTrue(new InstantCommand(() ->
    // statusSubsystem.lightLeft(0, 255, 128)));
    // m_operatorController.rightBumper().whileTrue(new InstantCommand(() ->
    // statusSubsystem.lightRight(60, 255, 128)));

    // Lock REEF
    m_driverController.leftBumper().whileTrue(new SwerveLockHeading(swerveSubsytem,
        () -> -m_driverController.getLeftY(),
        () -> -m_driverController.getLeftX(),
        () -> -m_driverController.getRightX(),
        Limelight.kReef));
    m_driverController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, 0.1)
    .whileTrue(new SwerveAutoGo(swerveSubsytem, Limelight.kReef, m_driverController::getLeftTriggerAxis));


    // Coral Grabeer Angle
    m_operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
    .whileTrue(new AngleNoraml(algaeGrabberAngleSubsystem, coralGrabberSubsystem, CoralState.kCoralUP, null));
    m_operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)
    .whileTrue(new AngleNoraml(algaeGrabberAngleSubsystem, coralGrabberSubsystem, CoralState.kCoralDown, null));

    // Elevator normal
    m_operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.1)
    .whileTrue(new ElevatorNormal(elevatorSubsystem, ElevatorState.kUP));
    m_operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1)
    .whileTrue(new ElevatorNormal(elevatorSubsystem, ElevatorState.kDown));

    // Algae Grabber Angle
    m_driverController.pov(0).whileTrue(new AngleNoraml(algaeGrabberAngleSubsystem, coralGrabberSubsystem, null, AlgaeState.kAlgaeUP));
    m_driverController.pov(180).whileTrue(new AngleNoraml(algaeGrabberAngleSubsystem, coralGrabberSubsystem, null, AlgaeState.kAlgaeDown));
    // Algae Grabber
    m_driverController.pov(90).toggleOnTrue(new GrabberNormal(coralGrabberSubsystem, algaeGrabberAngleSubsystem, null, AlgaeState.kputAlgae));
    m_driverController.pov(270).toggleOnTrue(new GrabberNormal(coralGrabberSubsystem, algaeGrabberAngleSubsystem, null, AlgaeState.kgetAlgae));

    // Coral Angle Auto
    m_operatorController.pov(0).whileTrue(new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, null, CoralState.kCoralTop, null));
    
    // Coral Grabber
    m_operatorController.rightTrigger().whileTrue(new GrabberNormal(coralGrabberSubsystem, algaeGrabberAngleSubsystem, CoralState.kCoralFwd, null));
    m_operatorController.leftTrigger().whileTrue(new GrabberNormal(coralGrabberSubsystem, algaeGrabberAngleSubsystem, CoralState.kCoralRev, null));

    // Coral Auto
    m_operatorController.start().onTrue(new CoralAuto(coralGrabberSubsystem));


    // Angle & Elevator All Auto
    m_operatorController.x().onTrue(new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL1, CoralState.kL1, AlgaeState.kAlgaeTop)); //L1
    m_operatorController.y().onTrue(new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL2, CoralState.kL2, AlgaeState.kAlgaeTop)); // L2
    m_operatorController.b().onTrue(new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL3, CoralState.kL3, AlgaeState.kAlgaeTop)); // L3
    m_operatorController.a().onTrue(new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL4, CoralState.kCoralTop, AlgaeState.kAlgaeTop)); // L4

    // Angle & Elevator All Defult
    m_driverController.y().onTrue(new AngleAutoDefult(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kDefault, CoralState.kCoralDefult, AlgaeState.kAlgaeDefult));

    // Proseccor
    m_driverController.a().onTrue(new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kDefault, CoralState.kCoralTop, AlgaeState.kAlgaeTop));

  }

  private void setDefaultCommand() {
    swerveSubsytem.setDefaultCommand(new SwerveFieldRelative(swerveSubsytem,
        () -> -m_driverController.getLeftY(), // X-Axis
        () -> -m_driverController.getLeftX(), // Y-Axis
        () -> -m_driverController.getRightX() // R-Axis
    ));

  }

  private void configureNamedCommands() {

    // Angle & Elevator State
    NamedCommands.registerCommand("AllDefult", 
    new AngleAutoDefult(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kDefault, CoralState.kCoralDefult, AlgaeState.kAlgaeDefult));
    NamedCommands.registerCommand("L1", 
    new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL1, CoralState.kL1, AlgaeState.kAlgaeTop));
    NamedCommands.registerCommand("L2", 
    new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL2, CoralState.kL2, AlgaeState.kAlgaeTop));
    NamedCommands.registerCommand("L3", 
    new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL3, CoralState.kL3, AlgaeState.kAlgaeTop));
    NamedCommands.registerCommand("L4", 
    new AngleAuto(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL4, CoralState.kCoralTop, AlgaeState.kAlgaeTop));

    // Get Point >w<
    NamedCommands.registerCommand("PutCoral", new GrabberStop(coralGrabberSubsystem, algaeGrabberAngleSubsystem, CoralState.kCoralRev, null));
    NamedCommands.registerCommand("CoralFwd", new GrabberStop(coralGrabberSubsystem, algaeGrabberAngleSubsystem, CoralState.kCoralFwd, null));
    NamedCommands.registerCommand("getAlgae", new GrabberStop(coralGrabberSubsystem, algaeGrabberAngleSubsystem, null, AlgaeState.kgetAlgae));
    NamedCommands.registerCommand("putAlgae", new GrabberStop(coralGrabberSubsystem, algaeGrabberAngleSubsystem, null, AlgaeState.kputAlgae));

    // Stop Algae & Coral
    NamedCommands.registerCommand("CoralStop", new GrabberStop(coralGrabberSubsystem, algaeGrabberAngleSubsystem, CoralState.kCoralStop, null));
    NamedCommands.registerCommand("AlgaeStop", new GrabberStop(coralGrabberSubsystem, algaeGrabberAngleSubsystem, null, AlgaeState.kAlgaeStop));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}