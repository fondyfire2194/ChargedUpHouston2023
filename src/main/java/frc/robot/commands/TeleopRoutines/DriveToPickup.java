// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPickup extends CommandBase {
  /** Creates a new DriveAndPickup. */
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_rate;
  private ProfiledPIDController m_pidController;
  private SimpleMotorFeedforward m_sff;
  private TrapezoidProfile.Constraints trapCon;
  private double lastSpeed = 0;
  private double lastTime;

  public DriveToPickup(DriveSubsystem drive, double distance, double rate) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_distance = distance;
    m_rate = rate;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    trapCon = new TrapezoidProfile.Constraints(m_rate, m_rate * 2);

    m_sff = new SimpleMotorFeedforward(.172, 3.16);

    m_pidController=new ProfiledPIDController(.1, 0, 0, trapCon);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double acceleration = (m_pidController.getSetpoint().velocity -
        lastSpeed) / (Timer.getFPGATimestamp() - lastTime);

    double ffVolts = m_sff.calculate(m_pidController.getSetpoint().velocity, acceleration);

    double pidVal = m_pidController.calculate(m_drive.getEstimatedPosition().getX(), m_distance);

    double ffMPS = ffVolts * DriveConstants.kMaxSpeedMetersPerSecond / 12;

    if (!m_drive.cubeFound)

      m_drive.drive(ffMPS + pidVal, 0, 0);

    else {

      double xError = m_pidController.calculate(m_drive.tx, -1.25);

      SmartDashboard.putNumber("XCUBERR", xError);

      m_drive.drive(ffMPS + pidVal, 0, xError);

    }

    lastSpeed = m_pidController.getSetpoint().velocity;

    lastTime = Timer.getFPGATimestamp();

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
