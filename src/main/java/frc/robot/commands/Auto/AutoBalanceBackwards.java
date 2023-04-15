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
  private double positionHeld = 0;

  public AutoBalanceBackwards(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    endCommand = false;
    positionHeld = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    double currentPitchDegrees = m_drive.getCompedGyroPitch();

    double currentPitchRadians = ((currentPitchDegrees * Math.PI) / 180);

    double motorMultiplier = -Math.sin(currentPitchRadians) * 1.8;

    double motorMPS = motorMultiplier * DriveConstants.kMaxSpeedMetersPerSecond;

    SmartDashboard.putNumber("motorMPS", motorMPS);
    // if (Math.abs(motorSpeed) > DriveConstants.kMaxSpeedMetersPerSecond) {
    // motorSpeed = Math.signum(motorSpeed) *
    // DriveConstants.kMaxSpeedMetersPerSecond;
    // }

    m_drive.drive(motorMPS, 0, 0);
    // if (motorMultiplier < 0.005 || motorMultiplier > -0.005) {

    // if (motorMultiplier < 0.01 && motorMultiplier > -0.01) {
    if (Math.abs(currentPitchDegrees) < 2.5) {
      positionHeld++;
      if (positionHeld > 40) {
        endCommand = true;
        m_drive.stopModules();
        m_drive.setX();
      }
    } else {
      positionHeld = 0;
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
