// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.Test;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightVision;

public class TrajectoryCorrectForCube extends CommandBase {
  /** Creates a new TrajectoryCorrectForCube. */
  private DriveSubsystem m_drive;
  private LimelightVision m_llv;
  private PIDController m_pidController;

  public TrajectoryCorrectForCube(DriveSubsystem drive, LimelightVision llv) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_llv = llv;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setCubeDetectorPipeline();

    m_pidController = new PIDController(1, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xError = m_pidController.calculate(m_drive.tx, -5);

    SmartDashboard.putNumber("XCUBERR", xError);

    double temp = xError;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drive.m_fieldOriented = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}