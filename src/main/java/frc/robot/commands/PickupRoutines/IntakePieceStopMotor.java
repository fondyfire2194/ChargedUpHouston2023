// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePieceStopMotor extends CommandBase {
  /** Creates a new GetPieceAtIntake. */
  private IntakeSubsystem m_intake;
  private double m_volts;
  private LinearFilter rpmfilter = LinearFilter.movingAverage(5);
  private double runningThresholdRPM = 500;
  boolean intakeAtSpeed;
  double currentRPM;
  private double stoppedThresholdRPM = 50;
  private double simTimer;

  public IntakePieceStopMotor(IntakeSubsystem intake, double volts) {
    m_intake = intake;
    m_volts = volts;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeAtSpeed = false;
    rpmfilter.reset();
    simTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setMotorVolts(m_volts);
    currentRPM = rpmfilter.calculate(m_intake.getRPM());
    if (currentRPM > runningThresholdRPM)
      intakeAtSpeed = true;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotorVolts(.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeAtSpeed && currentRPM < stoppedThresholdRPM
        || RobotBase.isSimulation() && Timer.getFPGATimestamp() > simTimer + 2;
  }
}
