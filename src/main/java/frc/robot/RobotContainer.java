// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAction;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberAngleAction;
import frc.robot.Constants.AlgaeGrabberConstants.AlgaeGrabberState;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberAction;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberAngleAction;
import frc.robot.Constants.CoralGrabberConstants.CoralGrabberState;
import frc.robot.Constants.ElevatorConstants.ElevatorAction;
import frc.robot.Constants.ElevatorConstants.ElevatorState;
import frc.robot.Constants.IntakeConstants.IntakeAction;
import frc.robot.Constants.IntakeConstants.IntakeAngleAction;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.commands.State.StateAuto;
import frc.robot.commands.State.StateAutoDefault;
import frc.robot.commands.Elevator.ElevatorAuto;
import frc.robot.commands.Elevator.ElevatorNormal;
import frc.robot.commands.Grabber.AlgaeAngleNormal;
import frc.robot.commands.Grabber.AlgaeGrabberNormal;
import frc.robot.commands.Grabber.CoralAngleNoraml;
import frc.robot.commands.Grabber.CoralGrabberAuto;
// import frc.robot.commands.Grabber.CoralGrabberAuto;
import frc.robot.commands.Grabber.CoralGrabberNormal;
import frc.robot.commands.Intake.IntakeAngleNormal;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeNormal;
import frc.robot.commands.Swerve.SwerveAutoGo;
import frc.robot.commands.Swerve.SwerveLockHeading;
import frc.robot.commands.Swerve.SwerveRobotRelative;
import frc.robot.commands.Swerve.SwerveSmartReef;
// import frc.robot.commands.Swerve.SwerveXMode;
import frc.robot.commands.Swerve.SwerveFieldRelative;
// import frc.robot.subsystems.StatusSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // private final StatusSubsystem statusSubsystem = new StatusSubsystem(9, 270);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  private static CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  private static XboxController m_driverControllerHID = new XboxController(OIConstants.kDriverControllerPort);


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
  

  // Define Commands for Controller binding, NamedCommand
  private final Command cmdAllDefault = new StateAutoDefault(algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem);

  private final Command cmdStateAutoL1 = new StateAuto(
    algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL1, CoralGrabberState.kL1, AlgaeGrabberState.kGetL2);
  private final Command cmdStateAutoL2 = new StateAuto(
    algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL2, CoralGrabberState.kL1, AlgaeGrabberState.kGetL2);
  private final Command cmdStateAutoL3 = new StateAuto(
    algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL3, CoralGrabberState.kL3, AlgaeGrabberState.kGetL3);
  private final Command cmdStateAutoL4 = new StateAuto(
    algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kL4, CoralGrabberState.kL4, AlgaeGrabberState.kGetL2);

  private final Command cmdStateAutoProseccor = new StateAuto(
    algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, ElevatorState.kDefault, CoralGrabberState.kL4, AlgaeGrabberState.kPutPro);
  private final Command cmdCoralAngleAutoL4 = new StateAuto(
    algaeGrabberAngleSubsystem, coralGrabberSubsystem, elevatorSubsystem, null, CoralGrabberState.kL4, null);

  

  private void configureBindings() {

    m_driverController.start().whileTrue(new InstantCommand(() -> swerveSubsytem.zeroHeading()));
   
    m_driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.2)
        .whileTrue(new SwerveSmartReef(swerveSubsytem, Limelight.kReef, m_driverController::getRightTriggerAxis));
    m_driverController.rightBumper().whileTrue(new SwerveSmartReef(swerveSubsytem, Limelight.kReef, null));


    // // Swerve Robot Relative
    // m_driverController.povCenter().whileFalse(new SwerveRobotRelative(swerveSubsytem, m_driverControllerHID::getPOV));

    // Intake Auto
    m_driverController.pov(0).onTrue(new IntakeAuto(intakeSubsystem, IntakeState.kDefult));
    m_driverController.pov(90).onTrue(new IntakeAuto(intakeSubsystem, IntakeState.kCoral));
    m_driverController.pov(180).onTrue(new IntakeAuto(intakeSubsystem, IntakeState.kAlgae));
    m_driverController.pov(270).onTrue(new IntakeAuto(intakeSubsystem, IntakeState.kIntake));

    //Intake Normal
    m_driverController.b().whileTrue(new IntakeNormal(intakeSubsystem, IntakeAction.kPut));
    m_driverController.x().whileTrue(new IntakeNormal(intakeSubsystem, IntakeAction.kGet));

    // Angle & Elevator All Default
    m_driverController.y().onTrue(cmdAllDefault);

    // Proseccor
    m_driverController.a().onTrue(cmdStateAutoProseccor);


    // Elevator Normal
    m_operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1)
        .whileTrue(new ElevatorNormal(elevatorSubsystem, ElevatorAction.kDown));
    m_operatorController.axisLessThan(XboxController.Axis.kRightY.value, -0.1)
        .whileTrue(new ElevatorNormal(elevatorSubsystem, ElevatorAction.kUP));

    // Coral Grabeer Angle Normal
    m_operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1)
        .whileTrue(new CoralAngleNoraml(coralGrabberSubsystem, CoralGrabberAngleAction.kUP));
    m_operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.1)
        .whileTrue(new CoralAngleNoraml(coralGrabberSubsystem, CoralGrabberAngleAction.kDown));

    // Intake Angle Noramal
    m_operatorController.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1)
        .whileTrue(new IntakeAngleNormal(intakeSubsystem, IntakeAngleAction.kUP));
    m_operatorController.axisLessThan(XboxController.Axis.kLeftX.value, -0.1)
        .whileTrue(new IntakeAngleNormal(intakeSubsystem, IntakeAngleAction.kDown));

    // Coral Grabber Normal
    m_operatorController.rightTrigger()
        .whileTrue(new CoralGrabberNormal(coralGrabberSubsystem, CoralGrabberAction.kFwd));
    m_operatorController.leftTrigger()
        .whileTrue(new CoralGrabberNormal(coralGrabberSubsystem, CoralGrabberAction.kRev));


    // Algae Grabber Normal
    m_operatorController.pov(90)
        .toggleOnTrue(new AlgaeGrabberNormal(algaeGrabberAngleSubsystem, AlgaeGrabberAction.kPut));
    m_operatorController.pov(270)
        .toggleOnTrue(new AlgaeGrabberNormal(algaeGrabberAngleSubsystem, AlgaeGrabberAction.kGet));


    // Coral Angle Auto
    m_operatorController.pov(0).onTrue(cmdCoralAngleAutoL4);

    // Coral Grabber Auto
    m_operatorController.leftBumper().onTrue(new CoralGrabberAuto(coralGrabberSubsystem));

    // Angle & Elevator All Auto
    m_operatorController.x().onTrue(cmdStateAutoL1); // L1
    m_operatorController.y().onTrue(cmdStateAutoL2); // L2
    m_operatorController.b().onTrue(cmdStateAutoL3); // L3
    m_operatorController.a().onTrue(cmdStateAutoL4); // L4

    // Elevator Default
    m_operatorController.rightBumper().onTrue(new ElevatorAuto(elevatorSubsystem, ElevatorState.kDefault));

  }

  private void setDefaultCommand() {
    swerveSubsytem.setDefaultCommand(new SwerveFieldRelative(swerveSubsytem,
        () -> -m_driverController.getLeftY(), // X-Axis
        () -> -m_driverController.getLeftX(), // Y-Axis
        () -> -m_driverController.getRightX() // R-Axis
    ));

  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Default", cmdAllDefault);
    NamedCommands.registerCommand("L1", cmdStateAutoL1);
    NamedCommands.registerCommand("L2", cmdStateAutoL2);
    NamedCommands.registerCommand("L3", cmdStateAutoL3);
    NamedCommands.registerCommand("L4", cmdStateAutoL4);
    NamedCommands.registerCommand("CGFwd", new CoralGrabberNormal(coralGrabberSubsystem, CoralGrabberAction.kFwd));
    NamedCommands.registerCommand("CGRev", new CoralGrabberNormal(coralGrabberSubsystem, CoralGrabberAction.kRev));
    NamedCommands.registerCommand("CGStop", new CoralGrabberNormal(coralGrabberSubsystem, CoralGrabberAction.kStop));
    NamedCommands.registerCommand("AGGet", new AlgaeGrabberNormal(algaeGrabberAngleSubsystem, AlgaeGrabberAction.kGet));
    NamedCommands.registerCommand("AGPut", new AlgaeGrabberNormal(algaeGrabberAngleSubsystem, AlgaeGrabberAction.kPut));
    NamedCommands.registerCommand("AGStop", new AlgaeGrabberNormal(algaeGrabberAngleSubsystem, AlgaeGrabberAction.kStop));
    NamedCommands.registerCommand("GetC", new CoralGrabberAuto(coralGrabberSubsystem));
    // NamedCommands.registerCommand("L4", cmdStateAutoL4);

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