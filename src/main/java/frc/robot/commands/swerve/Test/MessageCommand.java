// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MessageCommand extends InstantCommand {
  private String m_message;

  public MessageCommand(String message) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_message = message;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  SmartDashboard.putString("ActiveDrop", m_message);
  }
}
