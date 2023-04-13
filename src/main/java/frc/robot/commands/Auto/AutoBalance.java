// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private DriveSubsystem m_drive;
  private boolean m_forward;
  private double isBalancedStartTime;
  private boolean endCommand;
  private boolean onRamp;
  private LinearFilter pitchFilter = LinearFilter.movingAverage(10);// 200ms

  double isBalancedDegrees = 1;

  double isBalancedTime = .8;// secs

  private double pitchMultiplier;

  double motorSpeed;

  public AutoBalance(DriveSubsystem drive, boolean forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_forward = forward;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endCommand = false;
    isBalancedStartTime = 0;
    pitchFilter.reset();
    m_drive.setClosedLoop(true);
    onRamp = false;
    motorSpeed = 1;
    pitchMultiplier = 1.8;
    if (m_forward) {
      motorSpeed *= -1;
      pitchMultiplier *= -1;//
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentPitchDegrees = pitchFilter.calculate(m_drive.getCompedGyroPitch());

    double currentPitchRadians = Units.degreesToRadians(currentPitchDegrees);

    double speedMultiplier = Math.sin(currentPitchRadians) * pitchMultiplier;

    motorSpeed = speedMultiplier * DriveConstants.kMaxSpeedMetersPerSecond;

    SmartDashboard.putNumber("motorNum", speedMultiplier);

    m_drive.drive(motorSpeed, 0, 0);

    if (Math.abs(currentPitchDegrees) < isBalancedDegrees) {

      if (isBalancedStartTime == 0)
        isBalancedStartTime = Timer.getFPGATimestamp();

      if (isBalancedStartTime != 0 && Timer.getFPGATimestamp() > (isBalancedStartTime + isBalancedTime)) {
        endCommand = true;
        m_drive.stopModules();
        m_drive.setX();
      }

    } else {

      isBalancedStartTime = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endCommand;
  }
}
