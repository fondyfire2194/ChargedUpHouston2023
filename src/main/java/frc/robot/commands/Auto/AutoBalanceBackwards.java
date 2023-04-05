// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceBackwards extends CommandBase {
  /** Creates a new AutoBalance. */
  private DriveSubsystem m_drive;
  private double startTime;
  private double gyroStartPosition;
  private boolean endCommand;
  private double speedMultiplier = 1.5;

  public AutoBalanceBackwards(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    gyroStartPosition = m_drive.getGyroPitch();
    endCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    double currentGyro = m_drive.getGyroPitch();

    if ((currentTime - startTime) < 1.5) { // run full force backwards for 3 seconds??
      m_drive.drive(DriveConstants.kMaxSpeedMetersPerSecond * -0.8, 0, 0);
    } else {
      double currentPitch = currentGyro - gyroStartPosition;
      double currentPitchRadians = ((currentPitch * Math.PI) / 180);
      double motorMultiplier = Math.sin(currentPitchRadians);

      double motorSpeed = motorMultiplier * DriveConstants.kMaxSpeedMetersPerSecond * 1.7;

      SmartDashboard.putNumber("motorNum", motorMultiplier);
      // if (Math.abs(motorSpeed) > DriveConstants.kMaxSpeedMetersPerSecond) {
      // motorSpeed = Math.signum(motorSpeed) *
      // DriveConstants.kMaxSpeedMetersPerSecond;
      // }
      m_drive.drive(motorSpeed, 0, 0);
      // if (motorMultiplier < 0.05 || motorMultiplier > -0.05) {
      // endCommand = true;
      // m_drive.stopModules();
      // }
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
