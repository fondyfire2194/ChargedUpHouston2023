// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAprilTag extends CommandBase {
  /** Creates a new TurnToGamepiece. */
  private final DriveSubsystem m_drive;
  private double m_speed;
  private double m_angle;
  private final int m_id;
  private PIDController m_controller = new PIDController(.1, 0, 0);

  public TurnToAprilTag(DriveSubsystem drive, double speed, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_id = id;
    m_speed = speed;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_drive.hasTag || m_drive.hasTag && m_drive.fiducialID != m_id) {
      m_drive.drive(0, 0, m_speed);
    }

    if (m_drive.hasTag && m_drive.fiducialID == m_id) {
      double rotspeed = m_controller.calculate(m_drive.tx, 0);
      m_drive.drive(0, 0, rotspeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotBase.isSimulation() || m_drive.cubeFound && Math.abs(m_drive.tx) < .2;
  }
}
