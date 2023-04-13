// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.Test;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryCorrectForCube extends CommandBase {
  /** Creates a new TrajectoryCorrectForCube. */
  private DriveSubsystem m_drive;
  private ProfiledPIDController m_pidController;
  private TrapezoidProfile.Constraints trapCon;

  public TrajectoryCorrectForCube(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.clearAngleCorrection();
    trapCon = new TrapezoidProfile.Constraints(.5, 1);
    m_pidController = new ProfiledPIDController(.1, 0, 0, trapCon);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_drive.cubeFound)

      m_drive.clearAngleCorrection();

    else {

      double xError = m_pidController.calculate(m_drive.tx, -1.25);

      SmartDashboard.putNumber("XCUBERR", xError);

      double temp = xError;

      m_drive.setAngleCorrection(temp);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.clearAngleCorrection();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}