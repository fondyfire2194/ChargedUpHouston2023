// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.LiftArmSubsystem;

/** Add your docs here. */
public class MonitorThreadLift {

    private LiftArmSubsystem m_lift;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable liftprof = inst.getTable("liftprof");

    public DoublePublisher goalInches;
    public DoublePublisher velocity;
    public DoublePublisher feedforward;
    public DoublePublisher pidval;
    public DoublePublisher angle;
    public DoublePublisher profpos;
    public DoublePublisher disterr;
    public DoublePublisher volts;
    public DoublePublisher profvel;
    public DoublePublisher velerr;
    public DoublePublisher posnerr;
    public DoublePublisher amps;
    public DoublePublisher relenc;
    public DoublePublisher accel;

    public BooleanPublisher inizone;
    public BooleanPublisher atGoal;

    public MonitorThreadLift(LiftArmSubsystem lift) {

        m_lift = lift;

        goalInches = liftprof.getDoubleTopic("GOALINCHES").publish();
        velocity = liftprof.getDoubleTopic("ACTVELIPS").publish();
        angle = liftprof.getDoubleTopic("CANCODERDEG").publish();
        feedforward = liftprof.getDoubleTopic("FFWD").publish();
        pidval = liftprof.getDoubleTopic("PIDVAL").publish();
        profpos = liftprof.getDoubleTopic("PROFILEPOSN").publish();
        disterr = liftprof.getDoubleTopic("DISTERR").publish();
        volts = liftprof.getDoubleTopic("VOLTS").publish();
        profvel = liftprof.getDoubleTopic("PROFVEL").publish();
        inizone = liftprof.getBooleanTopic("INIZONE").publish();
        atGoal = liftprof.getBooleanTopic("ATGOAL").publish();
        velerr = liftprof.getDoubleTopic("VELERR").publish();
        posnerr = liftprof.getDoubleTopic("POSNERR").publish();
        amps = liftprof.getDoubleTopic("CURRENT").publish();
        relenc = liftprof.getDoubleTopic("RELENC").publish();
        accel = liftprof.getDoubleTopic("ACCELIPSSQD").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("LIFT Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {

                    m_lift.tstCtr++;

                    if (true) {

                        goalInches.set(m_lift.goalInches);
                        velocity.set(m_lift.getVelocity());
                        angle.set(m_lift.getCanCoderPosition());
                        feedforward.set(m_lift.ff);
                        pidval.set(m_lift.pidVal);
                        volts.set(m_lift.volts);
                        profpos.set(m_lift.m_liftController.getSetpoint().position);
                        disterr.set(m_lift.m_liftController.getPositionError());
                        profvel.set(m_lift.m_liftController.getSetpoint().velocity);
                        inizone.set(m_lift.inIZone);
                        atGoal.set(m_lift.m_liftController.atGoal());
                        velerr.set(m_lift.m_liftController.getVelocityError());
                        amps.set(m_lift.amps);
                        relenc.set(m_lift.positioninches);
                        accel.set(m_lift.acceleration);

                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}