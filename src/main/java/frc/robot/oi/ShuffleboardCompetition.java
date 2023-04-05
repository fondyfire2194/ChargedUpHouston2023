// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.oi;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.AutoFactory;

/** Add your docs here. */
public class ShuffleboardCompetition {

        private LimelightVision m_llv;

        private DriveSubsystem m_drive;

        private AutoFactory m_af;

        private IntakeSubsystem m_intake;

        public ShuffleboardCompetition(LimelightVision llv, DriveSubsystem drive,
                        AutoFactory af, LiftArmSubsystem lift, ExtendArmSubsystem ext, WristSubsystem wrist,
                        IntakeSubsystem intake) {
                String name = "Competition";
                m_llv = llv;
                m_af = af;
                m_drive = drive;
                m_intake = intake;

                ShuffleboardTab area1 = Shuffleboard.getTab(name);

                area1.add("StartPosition", m_af.m_startLocationChooser)
                                .withSize(2, 1)
                                .withPosition(0, 0);

                area1.add("DelayChooser", m_af.m_startDelayChooser)
                                .withSize(2, 1)
                                .withPosition(2, 0);

                area1.add("AutoChooser", m_af.m_autoChooser)
                                .withSize(2, 1)
                                .withPosition(0, 1);

                area1.add("AutoChooser1", m_af.m_autoChooser1)
                                .withSize(2, 1)
                                .withPosition(2, 1);

                area1.addNumber("RobotPosnX", () -> round2dp(m_drive.getEstimatedPosition().getTranslation().getX()))
                                .withSize(1, 1)
                                .withPosition(0, 2);
                area1.addNumber("RobotPosnY", () -> round2dp(m_drive.getEstimatedPosition().getTranslation().getY()))
                                .withSize(1, 1)
                                .withPosition(0, 3);
                area1.addNumber("Heading", () -> round2dp(m_drive.getHeadingDegrees()))
                                .withSize(1, 1)
                                .withPosition(0, 4);
                area1.addNumber("Pitch", () -> round2dp(m_drive.getGyroPitch()))
                                .withSize(1, 1)
                                .withPosition(1, 4);
                area1.addNumber("DriveSpeed", () -> round2dp(m_drive.m_frontLeft.getDriveVelocity()))
                                .withSize(1, 1)
                                .withPosition(2, 4);
                area1.addBoolean("AllCANOK", () -> m_drive.ALL_CANOK)
                                .withSize(2, 1)
                                .withPosition(2, 2);
                area1.addBoolean("FieldOriented", () -> m_drive.m_fieldOriented)
                                .withSize(2, 1)
                                .withPosition(2, 3);

        }

        public double round2dp(double number) {
                number = Math.round(number * 100);
                number /= 100;
                return number;
        }
}
