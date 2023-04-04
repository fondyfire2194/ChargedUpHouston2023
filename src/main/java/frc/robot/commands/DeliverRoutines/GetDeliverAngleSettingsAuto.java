// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

public class GetDeliverAngleSettingsAuto extends CommandBase {
  /** Creates a new DeliverAngleSettings. */
  private LiftArmSubsystem m_lift;
  private ExtendArmSubsystem m_ext;
  private WristSubsystem m_wrist;
  private IntakeSubsystem m_intake;

  private int m_level;

  private double m_liftInches;

  private double m_liftDegrees;

  private double m_extDistance;

  private double m_wristAngleRads;

  private boolean done;

  int ground = 0;
  int mid = 1;
  int top = 2;

  public GetDeliverAngleSettingsAuto(LiftArmSubsystem lift, ExtendArmSubsystem ext, WristSubsystem wrist,
      IntakeSubsystem intake, int level) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_ext = ext;
    m_wrist = wrist;
    m_intake = intake;
    m_level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    m_liftInches = 0;
    m_liftDegrees = presetLiftAngles.HOME.getAngle();
    m_extDistance = 0;
    m_wristAngleRads = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!done && m_level == ground) {
      done = true;

      m_liftInches = presetLiftAngles.PLACE_GROUND.getInches();
      m_liftDegrees = presetLiftAngles.PLACE_GROUND.getAngle();
      m_extDistance = presetExtArmDistances.PLACE_GROUND.getDistance();
      m_wristAngleRads = presetWristAngles.PLACE_GROUND.getAngleRads();
    }

    if (!done && m_level == mid) {
      done = true;

      m_liftInches = presetLiftAngles.PLACE_MID.getInches();
      m_liftDegrees = presetLiftAngles.PLACE_MID.getAngle();
      m_extDistance = presetExtArmDistances.PLACE_MID.getDistance();
      m_wristAngleRads = presetWristAngles.PLACE_MID.getAngleRads();
    }

    if (!done && m_level == top) {
      done = true;
      m_liftInches = presetLiftAngles.PLACE_TOP.getInches();
      m_liftDegrees = presetLiftAngles.PLACE_TOP.getAngle();
      m_extDistance = presetExtArmDistances.PLACE_TOP.getDistance();
      m_wristAngleRads = presetWristAngles.PLACE_TOP.getAngleRads();

    }

    m_lift.deliverInches = m_liftInches;
    m_lift.deliverAngle = m_liftDegrees;
    m_ext.deliverDistance = m_extDistance;
    m_wrist.deliverAngleRads = m_wristAngleRads;

    SmartDashboard.putNumber("EXRDELI", m_ext.deliverDistance);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
