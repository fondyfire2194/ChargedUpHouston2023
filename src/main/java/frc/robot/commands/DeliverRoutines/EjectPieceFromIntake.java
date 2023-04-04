// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSubsystem;

public class EjectPieceFromIntake extends CommandBase {
  /** Creates a new GetPieceAtIntake. */
  private IntakeSubsystem m_intake;
  private double m_volts;

  public EjectPieceFromIntake(IntakeSubsystem intake, double volts) {
    m_intake = intake;
    m_volts = volts;
    addRequirements(m_intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_intake.setMotorVolts(-m_volts);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotorVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
