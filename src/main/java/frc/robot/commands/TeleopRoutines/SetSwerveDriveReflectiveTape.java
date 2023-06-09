package frc.robot.commands.TeleopRoutines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class SetSwerveDriveReflectiveTape extends CommandBase {
  private final DriveSubsystem m_drive;
  private final LimelightVision m_llv;
  private PIDController m_controller = new PIDController(.1, 0, 0);
  private DoubleSupplier m_throttleInput;
  private DoubleSupplier m_rotationInput;

  

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDriveReflectiveTape(
      DriveSubsystem drive,
      LimelightVision llv, DoubleSupplier throttleInput, DoubleSupplier rotationInput) {
    m_drive = drive;
    m_llv = llv;
    m_throttleInput = throttleInput;
    m_rotationInput = rotationInput;

    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_llv.setHighTapePipeline();

    //m_drive.m_fieldOriented = false;

  };

  @Override
  public void execute() {
    SmartDashboard.putNumber("DriveX", m_drive.tx);
    SmartDashboard.putNumber("DriveY", m_drive.ty);
    Double throttle = MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()),
    DriverConstants.kControllerDeadband)
    * Math.signum(m_throttleInput.getAsDouble());
    Double rotation = MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()),
        DriverConstants.kControllerRotDeadband)
        * Math.signum(m_rotationInput.getAsDouble());
        throttle = Math.signum(throttle) * Math.pow(throttle, 2);
        rotation = Math.signum(rotation) * Math.pow(rotation, 2);
        throttle *= -DriveConstants.kMaxSpeedMetersPerSecond;
        rotation *= DriveConstants.kMaxRotationRadiansPerSecond;
        if (Math.abs(rotation) < DriverConstants.kControllerRotDeadband) 
           rotation = 0.;


    if (m_drive.hasTarget) {
      double speed = m_controller.calculate(m_drive.tx);
      m_drive.drive(throttle, -speed, rotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_drive.stopModules();
   m_llv.setHighTapePipeline();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
