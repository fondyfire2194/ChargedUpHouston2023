package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class SetSwerveDriveReflectiveTape extends CommandBase {
  private final DriveSubsystem m_drive;
  private final LimelightVision m_llv;
  private PIDController m_controller = new PIDController(.1, 0, 0);
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  public SetSwerveDriveReflectiveTape(
      DriveSubsystem drive,
      LimelightVision llv) {
    m_drive = drive;
    m_llv = llv;
    

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
    if (m_drive.hasTarget) {
      double speed = m_controller.calculate(m_drive.tx);
      m_drive.drive(0, -speed, 0);
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
