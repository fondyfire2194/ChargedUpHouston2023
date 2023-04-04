// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WaitWristAtTarget extends CommandBase {
  /** Creates a new WaitWristAtTarget. */
  private WristSubsystem m_wrist;
  private double m_startInRangeTime;
  private double m_startTime;
  private double m_range;
  private double m_inRangeTime;

  public WaitWristAtTarget(WristSubsystem wrist, double inRangeTime, double range) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wrist = wrist;
    m_inRangeTime = inRangeTime;
    m_range = range;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startInRangeTime = 0;
    m_startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_startInRangeTime == 0 && !m_wrist.inRange(m_range))
      m_startInRangeTime = 0;

    if (m_wrist.inRange(m_range) && m_startInRangeTime == 0) {
      m_startInRangeTime = Timer.getFPGATimestamp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist.atTargetAngle() || Timer.getFPGATimestamp() > m_startTime + 3
        || m_startInRangeTime != 0 && Timer.getFPGATimestamp() > m_startInRangeTime + m_inRangeTime;

  }
}
