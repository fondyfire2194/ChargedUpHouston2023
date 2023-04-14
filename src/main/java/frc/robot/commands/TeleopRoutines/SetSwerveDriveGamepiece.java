package frc.robot.commands.TeleopRoutines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class SetSwerveDriveGamepiece extends CommandBase {
  private final DriveSubsystem m_drive;
  private final LimelightVision m_llv;
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriverConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriverConstants.kTranslationSlew, -10000, 0);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriverConstants.kRotationSlew, -10000, 0);
  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  private final boolean m_cone;

  private boolean lowLevel;

  private double throttle;
  private double strafe;
  private double rotation;

  double loadY = 0;
  // double yTolerance = .1;
  private double kpY = .35;
  private double kpR = .01;
  private double endX;
  private final double stopDistance = 1;
  private PIDController m_controller = new PIDController(.05, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDriveGamepiece(
      DriveSubsystem drive,
      LimelightVision llv,
      boolean cone,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {
    m_drive = drive;
    m_llv = llv;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_throttleInput = throttleInput;
    m_cone = cone;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_cone)
      m_llv.setCubeDetectorPipeline();
    else
      m_llv.setConeDetectorPipeline();

  };

  @Override
  public void execute() {
    throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()),
        DriverConstants.kControllerDeadband)
        * Math.signum(m_throttleInput.getAsDouble());

    throttle *= -DriveConstants.kMaxSpeedMetersPerSecond / 3;

    double throttle_sl = m_slewX.calculate(throttle);

    strafe = MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()),
        DriverConstants.kControllerDeadband)
        * Math.signum(m_strafeInput.getAsDouble());

    rotation = MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()),
        DriverConstants.kControllerRotDeadband)
        * Math.signum(m_rotationInput.getAsDouble());

    throttle = Math.signum(throttle) * Math.pow(throttle, 2);
    strafe = Math.signum(strafe) * Math.pow(strafe, 2);
    rotation = Math.signum(rotation) * Math.pow(rotation, 2);

    throttle *= -DriveConstants.kMaxSpeedMetersPerSecond;
    strafe *= -DriveConstants.kMaxSpeedMetersPerSecond;
    rotation *= DriveConstants.kMaxRotationRadiansPerSecond;

    if (Math.abs(rotation) < DriverConstants.kControllerRotDeadband)
      rotation = 0;

    double strafe_sl = m_slewY.calculate(strafe);
    // double rotation_sl = m_slewRot.calculate(rotation);

    if (m_cone && m_drive.coneFound || !m_cone && m_drive.cubeFound) {

      double xError = m_controller.calculate(m_drive.tx, -1.25);

      SmartDashboard.putNumber("XCUBERR", xError);

      m_drive.drive(throttle_sl, 0, xError);

    } else {
      
      m_drive.drive(throttle_sl, strafe_sl, rotation);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
