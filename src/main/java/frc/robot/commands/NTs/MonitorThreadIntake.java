// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NTs;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.IntakeSubsystem;

/** Add your docs here. */
public class MonitorThreadIntake {

    private IntakeSubsystem m_intake;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    NetworkTable intprof = inst.getTable("intprof");

    public DoublePublisher rpm;
    public DoublePublisher amps;
    public DoublePublisher volts;
    public DoublePublisher batteryvolts;
    

    public MonitorThreadIntake(IntakeSubsystem intake) {

        m_intake = intake;

        rpm = intprof.getDoubleTopic("RPM").publish();

        amps = intprof.getDoubleTopic("CURRENT").publish();
 
        volts = intprof.getDoubleTopic("VOLTS").publish();

        batteryvolts = intprof.getDoubleTopic("BATTERYVOLTS").publish();

    }

    public void startThread() {

        // Set up thread properties and start it off
        monitorThread.setName("INT Thread");
        monitorThread.setPriority(Thread.MIN_PRIORITY);
        monitorThread.start();
    }

    Thread monitorThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                while (!Thread.currentThread().isInterrupted()) {

                    if (true) {

                        rpm.set(m_intake.getRPM());

                        amps.set(m_intake.getAmps());

                        volts.set(m_intake.voltage);

                        batteryvolts.set(RobotController.getBatteryVoltage());

                    }
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    });

}