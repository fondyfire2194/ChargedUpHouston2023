// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopRoutines;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToAngle extends ProfiledPIDCommand {
  /** Creates a new RotateToAngle. */
  private DriveSubsystem m_drive;

  public RotateToAngle(DriveSubsystem drive, double angle) {

    super(
        // The controller that the command will use
        drive.getRotatePID(),
        // This should return the measurement
        drive::getHeadingDegrees,
        // return setpoint
        angle,
        // use the output
        (output, setpoint) -> drive.drive(0, 0, output), drive);

    super.m_controller.setTolerance(1);

    super.m_controller.enableContinuousInput(-180, 180);


    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setIsRotating(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.m_controller.atSetpoint();
  }
}
