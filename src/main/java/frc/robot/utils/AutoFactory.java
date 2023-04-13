// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.AutoBalanceBackwards;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.DeliverRoutines.DeliverCubeFast;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.PickupRoutines.GroundIntakeUprightConePositions;
import frc.robot.commands.PickupRoutines.IntakePiece;
import frc.robot.commands.PickupRoutines.IntakePieceStopMotor;
import frc.robot.commands.TeleopRoutines.DriveToPickup;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLiftTravel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class AutoFactory {

    int sl_coop_0 = 0;
    int sl_noBumpShelf_1 = 1;
    int sl_bumpShelf_2 = 2;

    int as_doNothing_0 = 0;
    int as_pushCube_1 = 1;
    int as_deliverMid_2 = 2;
    int as_deliverTop_3 = 3;

    int as1_driveThruChargeAndBalance_1 = 1;
    int as1_secondCube_2 = 2;

    List<PathPlannerTrajectory> pathGroup;

    public Command autonomousCommand = new DoNothing();

    public final SendableChooser<Integer> m_autoChooser = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_autoChooser1 = new SendableChooser<Integer>();

    public final SendableChooser<Integer> m_startLocationChooser = new SendableChooser<Integer>();

    public final SendableChooser<Double> m_startDelayChooser = new SendableChooser<Double>();

    private DriveSubsystem m_drive;

    private LiftArmSubsystem m_lift;

    private ExtendArmSubsystem m_extend;

    private WristSubsystem m_wrist;

    private IntakeSubsystem m_intake;

    private TrajectoryFactory m_tf;

    private PathPlannerTrajectory traj1;

    private Command command0 = new DoNothing();

    private Command command1 = new DoNothing();

    private Command command2 = new DoNothing();

    public String traj1name = "PushCubeCenter";

    private boolean traj1Reqd;

    public AutoFactory(DriveSubsystem drive, LiftArmSubsystem lift, ExtendArmSubsystem extend,
            WristSubsystem wrist, IntakeSubsystem intake, TrajectoryFactory tf) {

        m_drive = drive;

        m_lift = lift;

        m_extend = extend;

        m_wrist = wrist;

        m_intake = intake;

        m_tf = tf;

        m_startDelayChooser.setDefaultOption("Zero Seconds", 0.);
        m_startDelayChooser.addOption("One Second", 1.);
        m_startDelayChooser.addOption("Two Seconds", 2.);
        m_startDelayChooser.addOption("Three Second", 3.);
        m_startDelayChooser.addOption("Four Seconds", 4.);
        m_startDelayChooser.addOption("Five Seconds", 4.);

        m_startLocationChooser.setDefaultOption("Coop", 0);

        m_startLocationChooser.addOption("NoBumpShelf", 1);

        m_startLocationChooser.addOption("BumpShelf", 2);

        m_autoChooser.setDefaultOption("Do Nothing", 0);

        m_autoChooser.addOption("PushCube", 1);

        m_autoChooser.addOption("DeliverMid", 2);

        m_autoChooser.addOption("DeliverTop", 3);

        m_autoChooser1.setDefaultOption("Do Nothing", 0);

        m_autoChooser1.addOption("ThroughCharge+Balance", 1);

        m_autoChooser1.addOption("PickupScore2ndCube", 2);

    }

    public Command getCommand1() {

        Command tempCommand = new DoNothing();

        traj1Reqd = false;

        int startLocation = m_startLocationChooser.getSelected();

        int autoselect = m_autoChooser.getSelected();

        if (startLocation == sl_coop_0 && autoselect == as_pushCube_1) {
            traj1name = "PushCubeCenter";
            traj1Reqd = true;
        }

        if (startLocation == sl_noBumpShelf_1 && autoselect == as_pushCube_1) {
            traj1name = "PushCubeLeftShelf";
            traj1Reqd = true;
        }

        if (startLocation == sl_bumpShelf_2 && autoselect == as_pushCube_1) {
            traj1name = "PushCubeRightShelf";
            traj1Reqd = true;
        }

        if (traj1Reqd) {
            traj1 = m_tf.getPathPlannerTrajectory(traj1name, 2, 1, false);
            tempCommand = m_tf.followTrajectoryCommand(traj1, traj1Reqd).withTimeout(8);
        }

        if (autoselect == as_deliverMid_2) {
            tempCommand = new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false);
        }

        if (autoselect == as_deliverTop_3) {
            tempCommand = new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, true);
        }
        return tempCommand;
    }

    public Command getCommand2() {

        Command tempCommand = new DoNothing();

        int startLocation = m_startLocationChooser.getSelected();

        int autoselect1 = m_autoChooser1.getSelected();

        if (startLocation == sl_coop_0) {// any of the coop starts
            if (autoselect1 == as1_driveThruChargeAndBalance_1) {
              //  gy = m_drive.getGyroPitch();
                List<PathPlannerTrajectory> balanceCommandList = m_tf.getPathPlannerTrajectoryGroup("BackUpCenter", 2.2,
                        7, false);
                tempCommand = new SequentialCommandGroup(

                        m_tf.followTrajectoryCommand(balanceCommandList.get(0), true),
                        new WaitCommand(1.5),
                        m_tf.followTrajectoryCommand(balanceCommandList.get(1), false),
                        new AutoBalanceBackwards(m_drive));
            }

        }

        if (startLocation == sl_noBumpShelf_1 && autoselect1 == as1_secondCube_2) {

            PathPlannerTrajectory eventPath = m_tf.getPathPlannerTrajectory("LeftShelfToLeftCubeRotate", 2, 1, false);
            HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("DropIntake1", new GroundIntakeUprightConePositions(m_lift, m_wrist, m_extend, m_intake)
                    .withTimeout(5));
            eventMap.put("RunIntake1", new IntakePieceStopMotor(m_intake, 11).withTimeout(4));
            FollowPathWithEvents eventCommand = new FollowPathWithEvents(m_tf.followTrajectoryCommand(eventPath, true),
                    eventPath.getMarkers(), eventMap);

            tempCommand = eventCommand;
        }

        if (startLocation == sl_bumpShelf_2 && autoselect1 == as1_secondCube_2) {

            List<PathPlannerTrajectory> eventPaths = m_tf.getPathPlannerTrajectoryGroup("RightSideSecondCube", 2, 1,
                    false);
            HashMap<String, Command> eventMap1 = new HashMap<>();
            eventMap1.put("DropIntake1", new GroundIntakeUprightConePositions(m_lift, m_wrist, m_extend, m_intake)
                    .withTimeout(5));
            eventMap1.put("RunIntake1", new IntakePiece(m_intake, 11).withTimeout(6));

            HashMap<String, Command> eventMap2 = new HashMap<>();
            eventMap2.put("HomePosition", new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist)
                    .withTimeout(5));
            eventMap2.put("Eject1", new EjectPieceFromIntake(m_intake, 12).withTimeout(1));

            PathPlannerTrajectory eventPath1 = eventPaths.get(0);
            PathPlannerTrajectory eventPath2 = eventPaths.get(1);

            FollowPathWithEvents eventCommand1 = new FollowPathWithEvents(
                    m_tf.followTrajectoryCommand(eventPath1, true),
                    eventPath1.getMarkers(), eventMap1);
            FollowPathWithEvents eventCommand2 = new FollowPathWithEvents(
                    m_tf.followTrajectoryCommand(eventPath2, false),
                    eventPath2.getMarkers(), eventMap2);

            tempCommand = new SequentialCommandGroup(eventCommand1, new WaitCommand(0.5), eventCommand2,
                    new EjectPieceFromIntake(m_intake, 12).withTimeout(1));

        }

        return tempCommand;
    }

    public void createCommands() {

        command0 = new DoNothing();
        command1 = new DoNothing();
        command2 = new DoNothing();

        command1 = getCommand1();
        command2 = getCommand2();

    }

    public Command getAutonomousCommand() {

        createCommands();

        autonomousCommand = new SequentialCommandGroup(command0, command1, command2);

        return autonomousCommand;

    }

    public Command bumpShelfAlt(DriveSubsystem drive, IntakeSubsystem intake, LiftArmSubsystem lift,
            ExtendArmSubsystem extend, WristSubsystem wrist) {

        Command temp = new DoNothing();

        Command one = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DriveToPickup(drive, 4.5, 3),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new ParallelCommandGroup(
                                        new GroundIntakeUprightConePositions(lift, wrist, extend, intake),
                                        new IntakePieceStopMotor(intake, 11)))),
                new RetractWristExtendLiftTravel(lift, extend, wrist));

        Command two = new SequentialCommandGroup(
                Commands.runOnce(() -> drive.resetOdometry(new Pose2d())),
                new DriveToPickup(drive, 4.5, 3),
                new DeliverCubeFast(lift, wrist, intake, extend, true));

        return new SequentialCommandGroup(one, two);

    }

}
