// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LiftArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftArmSubsystem;

public class WaitLiftAtTarget extends CommandBase {
  /** Creates a new WaitWristAtTarget. */
  private LiftArmSubsystem m_lift;
  private double m_startInRangeTime;
  private double m_maxTime;
  private double m_range;
  private double m_inRangeTime;
  private double maxStartTime;

  public WaitLiftAtTarget(LiftArmSubsystem lift, double inRangeTime, double range, double maxTime) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_inRangeTime = inRangeTime;
    m_range = range;
    m_maxTime = maxTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startInRangeTime = 0;
    maxStartTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_lift.inRange(m_range))
      m_startInRangeTime = 0;

    if (m_lift.inRange(m_range) && m_startInRangeTime == 0) {
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
    return m_lift.atTargetPosition() || Timer.getFPGATimestamp() > maxStartTime + m_maxTime
        || m_startInRangeTime != 0 && Timer.getFPGATimestamp() > m_startInRangeTime + m_inRangeTime;
  }
}
