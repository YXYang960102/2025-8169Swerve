package frc.robot.commands.Swerve;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsytem;

public class SwerveAutoGo extends Command {
  private final SwerveSubsytem swerveSubsystem;
  private final Limelight limelight;
  private final PIDController pidController = new PIDController(0.006, 0.005, 0);
  private final DoubleSupplier speedSup;
  private boolean detected;
  private final boolean useVision; // 是否使用視覺修正

  /** 
   * Creates a new SwerveAutoGo. 
   * @param swerveSubsystem  Swerve 
   * @param limelight        Limelight 
   * @param speedSup         Robot Speed
   * @param useVision        April Tag True or false
   */
  public SwerveAutoGo(SwerveSubsytem swerveSubsystem, Limelight limelight, DoubleSupplier speedSup, boolean useVision) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.speedSup = speedSup;
    this.useVision = useVision;

    pidController.setIZone(5);
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    detected = false;
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed = OIConstants.deadbandHandler(speedSup != null ? speedSup.getAsDouble() : 0, 0.4) * 0.2;
    double targetAngle = 0; 
    double turningCorrection = 0;

    if (LimelightHelpers.getTV(limelight.hostname) && useVision) {
      detected = true;
      targetAngle = AprilTagConstants.ID2Angle[(int) LimelightHelpers.getFiducialID(limelight.hostname) - 1];
      turningCorrection = pidController.calculate(LimelightHelpers.getTX(limelight.hostname), 0);
    }

    // 繼續路徑行進，若偵測到目標才會進行修正
    swerveSubsystem.setChassisOutput(speed * limelight.approachingXSpeed, turningCorrection, targetAngle, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    // 若 useVision 為 false，則永遠不因 Limelight 失去偵測而結束
    return false;
  }
}
