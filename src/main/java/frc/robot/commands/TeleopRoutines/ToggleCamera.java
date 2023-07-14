// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimelightVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ToggleCamera extends InstantCommand {
  private LimelightVision m_llv;

  public ToggleCamera(LimelightVision llv) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_llv = llv;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_llv.activeLimelight == 0)
      m_llv.activeLimelight = 1;
    else
      m_llv.activeLimelight = 0;
  }
}
