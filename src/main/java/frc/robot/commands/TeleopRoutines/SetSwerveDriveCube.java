package frc.robot.commands.TeleopRoutines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.LimelightVision;

public class SetSwerveDriveCube extends CommandBase {
  private final DriveSubsystem m_drive;
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriverConstants.kTranslationSlew);
  private final DoubleSupplier m_throttleInput;

  private boolean lowLevel;

  private double throttle;
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
  public SetSwerveDriveCube(
      DriveSubsystem drive,
      LimelightVision llv,
      GameHandlerSubsystem ghs,
      DoubleSupplier throttleInput) {
    m_drive = drive;

    m_throttleInput = throttleInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  };

  @Override
  public void execute() {
    throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()),
        DriverConstants.kControllerDeadband)
        * Math.signum(m_throttleInput.getAsDouble());

    // square values after deadband while keeping original sign

    throttle *= -DriveConstants.kMaxSpeedMetersPerSecond / 3;

    double throttle_sl = m_slewX.calculate(throttle);

    if (!m_drive.cubeFound)

      m_drive.drive(throttle_sl, 0, 0);

    else {

      double xError = m_controller.calculate(m_drive.tx, 0);

      m_drive.drive(throttle_sl, 0, 0);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_drive.hasTarget;
  }
}
