// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ExtendArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendArmSubsystem;

public class WaitExtendAtTarget extends CommandBase {
  /** Creates a new WaitWristAtTarget. */
  private ExtendArmSubsystem m_extend;
  private double m_startInRangeTime;
  private double m_range;
  private double m_inRangeTime;

  public WaitExtendAtTarget(ExtendArmSubsystem extend, double inRangeTime, double range) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_extend = extend;
    m_inRangeTime = inRangeTime;
    m_range = range;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startInRangeTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_extend.inRange(m_range)) {
      m_startInRangeTime = 0;
    }

    if (m_extend.inRange(m_range) && m_startInRangeTime == 0) {
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
    return m_extend.atTargetPosition()
        || m_startInRangeTime != 0 && Timer.getFPGATimestamp() > m_startInRangeTime + m_inRangeTime;

  }
}