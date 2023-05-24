// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.DriveSubsystem;


/** Add your docs here. */
public class MonitorThreadBackRight {

    private DriveSubsystem m_drive;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable prof = inst.getTable("backright");

    public BooleanPublisher openLoop;
    public DoublePublisher commandAngle;
    public DoublePublisher actualAngle;
    public DoublePublisher angleCommandVelocity;
    public DoublePublisher angleActualVelocity;
    public DoublePublisher driveCommandVelocity;
    public DoublePublisher driveActualVelocity;
    public IntegerPublisher driveAmps;
    public IntegerPublisher turnAmps;
    public BooleanPublisher angleInPosition;

    public MonitorThreadBackRight(DriveSubsystem drive) {

        m_drive = drive;
        commandAngle = prof.getDoubleTopic("COMMANDANGLE").publish();
        actualAngle = prof.getDoubleTopic("ACTANGLE").publish();
        angleCommandVelocity = prof.getDoubleTopic("CMDANGVEL").publish();
        angleActualVelocity = prof.getDoubleTopic("ACTANGVEL").publish();
        driveCommandVelocity = prof.getDoubleTopic("CMDDRVVEL").publish();
        driveActualVelocity = prof.getDoubleTopic("ACTDRVVEL").publish();
        driveAmps = prof.getIntegerTopic("DRIVEAMPS").publish();
        turnAmps = prof.getIntegerTopic("TURNAMPS").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("FR Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {

                    if (true) {
                        commandAngle.set(m_drive.m_backRight.angle);
                        actualAngle.set(m_drive.m_backRight.getTurnAngleDegs());
                        angleCommandVelocity.set(m_drive.m_backRight.getTurnAppliedOutput());
                        angleActualVelocity.set(m_drive.m_backRight.getTurnVelocity());
                        driveCommandVelocity.set(m_drive.m_backRight.getDriveSpeedSetpoint());
                        driveActualVelocity.set(m_drive.m_backRight.getDriveVelocity());
                        driveAmps.set((int) m_drive.m_backRight.getDriveCurrent());
                        turnAmps.set((int) m_drive.m_backRight.getTurnCurrent());

                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}