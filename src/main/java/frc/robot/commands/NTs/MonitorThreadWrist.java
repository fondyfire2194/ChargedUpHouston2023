// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class MonitorThreadWrist {

    private WristSubsystem m_wrist;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable wristprof = inst.getTable("wristprof");

    public DoublePublisher goalangle;
    public DoublePublisher velocity;
    public DoublePublisher angle;
    public DoublePublisher feedforward;
    public DoublePublisher pidval;
    public DoublePublisher profpos;
    public DoublePublisher disterr;
    public DoublePublisher volts;
    public DoublePublisher profvel;
    public BooleanPublisher inizone;
    public BooleanPublisher atGoal;
    public DoublePublisher velerr;
    public DoublePublisher amps;
    public DoublePublisher gravval;

    public MonitorThreadWrist(WristSubsystem wrist) {

        m_wrist = wrist;
        goalangle = wristprof.getDoubleTopic("GOALANGLE").publish();
        velocity = wristprof.getDoubleTopic("ACTVEL").publish();
        angle = wristprof.getDoubleTopic("ACTANGLE").publish();
        feedforward = wristprof.getDoubleTopic("FFWD").publish();
        pidval = wristprof.getDoubleTopic("PIDVAL").publish();
        profpos = wristprof.getDoubleTopic("PROFILEPOSN").publish();
        disterr = wristprof.getDoubleTopic("DISTERR").publish();
        volts = wristprof.getDoubleTopic("VOLTS").publish();
        profvel = wristprof.getDoubleTopic("PROFVEL").publish();
        inizone = wristprof.getBooleanTopic("INIZONE").publish();
        atGoal = wristprof.getBooleanTopic("ATGOAL").publish();
        velerr = wristprof.getDoubleTopic("VELERR").publish();
        amps = wristprof.getDoubleTopic("CURRENT").publish();
        gravval = wristprof.getDoubleTopic("GRAVVALL").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("WRIST Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {

        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {
                    m_wrist.tstCtr++;

                    if (true) {

                        goalangle.set(m_wrist.goalAngleRadians);
                        velocity.set(m_wrist.radspersec);
                        angle.set(m_wrist.angleRadians);
                        feedforward.set(m_wrist.ff);
                        pidval.set(m_wrist.pidVal);
                        volts.set(m_wrist.volts);
                        profpos.set(m_wrist.m_wristController.getSetpoint().position);
                        disterr.set(m_wrist.m_wristController.getPositionError());
                        velerr.set(m_wrist.m_wristController.getVelocityError());
                        profvel.set(m_wrist.m_wristController.getSetpoint().velocity);
                        inizone.set(m_wrist.inIZone);
                        atGoal.set(m_wrist.m_wristController.atGoal());
                        amps.set(m_wrist.getAmps());
                        gravval.set(m_wrist.gravVal);
                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}