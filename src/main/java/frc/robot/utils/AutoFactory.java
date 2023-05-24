// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.commands.Auto.AutoBalance;
import frc.robot.commands.Auto.AutoBalanceBackwards;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.DeliverRoutines.DeliverCubeFast;
import frc.robot.commands.DeliverRoutines.DeliverPiecePositions;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettingsAuto;
import frc.robot.commands.PickupRoutines.GroundIntakeCubePositions;
import frc.robot.commands.PickupRoutines.GroundIntakeUprightConePositions;
//import frc.robot.commands.PickupRoutines.IntakePiece;
import frc.robot.commands.PickupRoutines.IntakePieceStopMotor;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLiftTravel;
import frc.robot.commands.TeleopRoutines.SetSwerveDriveGamepiece;
import frc.robot.commands.TeleopRoutines.TurnToAngle;
import frc.robot.commands.TeleopRoutines.TurnToGamepiece;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsBlue;
import frc.robot.subsystems.GameHandlerSubsystem.fieldTagsRed;

/** Add your docs here. */
public class AutoFactory {

        int sl_coopShelf_0 = 0;
        int sl_coopPipe_1 = 1;
        int sl_noBumpShelf_2 = 2;
        int sl_bumpShelf_3 = 3;

        int as_doNothing_0 = 0;
        int as_pushCube_1 = 1;
        int as_deliverMid_2 = 2;
        int as_deliverTop_3 = 3;

        int as1_driveThruChargeAndBalance_1 = 1;
        int as1_secondCube_2 = 2;
        int as1_balance_3 = 3;
        int as1_secondCubeAlt_4 = 4;
        int as_1_driveOutZone_5 = 5;

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

        private LimelightVision m_llv;

        private GameHandlerSubsystem m_ghs;

        private PathPlannerTrajectory traj1;

        private Command command0 = new DoNothing();

        private Command command1 = new DoNothing();

        private Command command2 = new DoNothing();

        public String traj1name = "PushCubeCenter";

        private boolean traj1Reqd;

        private List<PathPlannerTrajectory> noBumpStartTrajs;

        private List<PathPlannerTrajectory> noBumpStartTrajsAlt;

        private List<PathPlannerTrajectory> bumpStartTrajsAlt;

        List<PathPlannerTrajectory> balanceCommandList;

        List<PathPlannerTrajectory> bumpEventPathsList;

        public Pose2d endPose = new Pose2d();

        public AutoFactory(DriveSubsystem drive, LiftArmSubsystem lift, ExtendArmSubsystem extend,
                        WristSubsystem wrist, IntakeSubsystem intake, TrajectoryFactory tf, LimelightVision llv,
                        GameHandlerSubsystem ghs) {

                m_drive = drive;

                m_lift = lift;

                m_extend = extend;

                m_wrist = wrist;

                m_intake = intake;

                m_tf = tf;

                m_llv = llv;

                m_ghs = ghs;

                m_startDelayChooser.setDefaultOption("Zero Seconds", 0.);
                m_startDelayChooser.addOption("One Second", 1.);
                m_startDelayChooser.addOption("Two Seconds", 2.);
                m_startDelayChooser.addOption("Three Second", 3.);
                m_startDelayChooser.addOption("Four Seconds", 4.);
                m_startDelayChooser.addOption("Five Seconds", 4.);

                m_startLocationChooser.setDefaultOption("CoopShelf", 0);

                m_startLocationChooser.addOption("CoopPipe", 1);

                m_startLocationChooser.addOption("NoBumpShelf", 2);

                m_startLocationChooser.addOption("BumpShelf", 3);

                m_autoChooser.setDefaultOption("Do Nothing", 0);

                m_autoChooser.addOption("PushCube", 1);

                m_autoChooser.addOption("DeliverMid", 2);

                m_autoChooser.addOption("DeliverTop", 3);

                m_autoChooser1.setDefaultOption("Do Nothing", 0);

                m_autoChooser1.addOption("ThroughCharge+Balance", 1);

                m_autoChooser1.addOption("PickupScore2ndCube", 2);

                m_autoChooser1.addOption("Balance", 3);

                m_autoChooser1.addOption("PickupScore2ndCubeAlt", 4);

                m_autoChooser1.addOption("DriveOutZone", 5);

                noBumpStartTrajs = m_tf.getPathPlannerTrajectoryGroup("NoBumpSide",
                                2.5, 4.0, false);

                noBumpStartTrajsAlt = m_tf.getPathPlannerTrajectoryGroup("NoBumpSideAlt",
                                2.5, 4.0, false);

                bumpStartTrajsAlt = m_tf.getPathPlannerTrajectoryGroup("BumpSideAlt",
                                2.5, 4.0, false);

                balanceCommandList = m_tf.getPathPlannerTrajectoryGroup("BackUpCenter", 2.2,
                                7, false);

                bumpEventPathsList = m_tf.getPathPlannerTrajectoryGroup("BumpSide", 2, 1,
                                false);

        }

        public Command getCommand1() {

                Command tempCommand = new DoNothing();

                traj1Reqd = false;

                int startLocation = m_startLocationChooser.getSelected();

                int autoselect = m_autoChooser.getSelected();

                if (startLocation == sl_coopShelf_0 && autoselect == as_pushCube_1) {
                        traj1name = "PushCubeCenter";
                        traj1Reqd = true;
                }

                if (startLocation == sl_noBumpShelf_2 && autoselect == as_pushCube_1) {
                        traj1name = "PushCubeLeftShelf";
                        traj1Reqd = true;
                }

                if (startLocation == sl_bumpShelf_3 && autoselect == as_pushCube_1) {
                        traj1name = "PushCubeRightShelf";
                        traj1Reqd = true;
                }

                if (traj1Reqd) {
                        traj1 = m_tf.getPathPlannerTrajectory(traj1name, 2, 1, false);
                        tempCommand = m_tf.followTrajectoryCommand(traj1, traj1Reqd).withTimeout(8);
                }

                if ((startLocation == sl_coopShelf_0 || startLocation == sl_noBumpShelf_2)
                                && autoselect == as_deliverMid_2) {
                        tempCommand = new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false);
                }

                if ((startLocation == sl_coopShelf_0 || startLocation == sl_noBumpShelf_2)
                                && autoselect == as_deliverTop_3) {
                        tempCommand = new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, true);
                }

                if (startLocation == sl_coopPipe_1 && autoselect == as_deliverMid_2) {
                        tempCommand = new SequentialCommandGroup(
                                        new GetDeliverAngleSettingsAuto(m_lift, m_extend, m_wrist, m_intake, 1),
                                        new DeliverPiecePositions(m_lift, m_extend, m_wrist, m_intake));
                }

                if (startLocation == sl_coopPipe_1 && autoselect == as_deliverTop_3) {
                        tempCommand = new SequentialCommandGroup(
                                        new GetDeliverAngleSettingsAuto(m_lift, m_extend, m_wrist, m_intake, 2),
                                        new DeliverPiecePositions(m_lift, m_extend, m_wrist, m_intake));
                }

                return tempCommand;
        }

        public Command getCommand2() {

                Command tempCommand = new DoNothing();

                int startLocation = m_startLocationChooser.getSelected();

                int autoselect1 = m_autoChooser1.getSelected();

                if (startLocation == sl_coopShelf_0 || startLocation == sl_coopPipe_1) {// any of the coop starts
                        if (autoselect1 == as1_driveThruChargeAndBalance_1) {
                                // gy = m_drive.getGyroPitch();
                                //
                                tempCommand = new SequentialCommandGroup(

                                                m_tf.followTrajectoryCommand(balanceCommandList.get(0), true),
                                                new WaitCommand(1.5),
                                                m_tf.followTrajectoryCommand(balanceCommandList.get(1), false),
                                                new AutoBalanceBackwards(m_drive));
                        }

                }

                if (startLocation == sl_noBumpShelf_2 && autoselect1 == as1_secondCube_2) {

                        SequentialCommandGroup eventCommand = new SequentialCommandGroup(
                                        Commands.runOnce(() -> m_lift.setController(
                                                        LiftArmConstants.liftArmFastConstraints, 1, false)),
                                        m_tf.followTrajectoryCommand(noBumpStartTrajs.get(0), true),
                                        new WaitCommand(.1),
                                        // new TurnToGamepiece(m_drive, -.25, true),
                                        new GroundIntakeCubePositions(m_lift, m_wrist, m_extend, m_intake),
                                        new WaitCommand(.1),
                                        m_tf.followTrajectoryCommand(noBumpStartTrajs.get(1), false)
                                                        .raceWith(new IntakePieceStopMotor(m_intake, 11)),
                                        new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist),
                                        m_tf.followTrajectoryCommand(noBumpStartTrajs.get(2), false),
                                        new EjectPieceFromIntake(m_intake, 11).withTimeout(1));

                        tempCommand = eventCommand;
                }
                /**
                 * Blue drive station no bump shelf is tag Id 6
                 * Red drive station no bump shelf is tag Id 3
                 * 
                 */

                boolean runTraj = false;

                if (startLocation == sl_noBumpShelf_2 && autoselect1 == as1_secondCubeAlt_4) {

                        if (runTraj) {

                                tempCommand = new SequentialCommandGroup(
                                                Commands.runOnce(() -> m_lift.setController(
                                                                LiftArmConstants.liftArmFastConstraints, 1, false)),
                                                m_tf.followTrajectoryCommand(noBumpStartTrajsAlt.get(0), true),
                                                new WaitCommand(.1),
                                                new GroundIntakeCubePositions(m_lift, m_wrist, m_extend, m_intake),
                                                // new TurnToGamepiece(m_drive, autoselect1, true),
                                                // new WaitCommand(.1),
                                                m_tf.followTrajectoryCommand(noBumpStartTrajsAlt.get(1), false)
                                                                .raceWith(new IntakePieceStopMotor(m_intake, 11)),
                                                new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist),
                                                m_tf.followTrajectoryCommand(noBumpStartTrajsAlt.get(2), false),
                                                new EjectPieceFromIntake(m_intake, 11).withTimeout(1));

                        }

                        else {
                                PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("NoBumpDriveOut", 2.2, 4,
                                                false);

                                endPose = fieldTagsBlue.BLUE_NO_BUMP.getPose();

                                if (DriverStation.getAlliance() == Alliance.Red)

                                        endPose = fieldTagsRed.RED_NO_BUMP.getPose();

                                // need to move x to be short of tag by ?? 1 meters

                                endPose.transformBy(new Transform2d(new Translation2d(-1, 0), new Rotation2d()));

                                tempCommand = new SequentialCommandGroup(

                                                Commands.runOnce(() -> m_lift.setController(
                                                                LiftArmConstants.liftArmFastConstraints, 1, false)),

                                                m_tf.followTrajectoryCommand(traj2, true).withTimeout(3),

                                                new WaitCommand(.1),

                                                new TurnToGamepiece(m_drive, -2, true),

                                                new GroundIntakeCubePositions(m_lift, m_wrist, m_extend, m_intake)
                                                                .withTimeout(5),

                                                new ParallelRaceGroup(

                                                                new SetSwerveDriveGamepiece(m_drive, m_llv, false,
                                                                                () -> -.5,
                                                                                () -> 0, () -> 0).withTimeout(5),
                                                                new IntakePieceStopMotor(m_intake,
                                                                                11).withTimeout(4)),

                                                new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist)
                                                                .withTimeout(4),

                                                Commands.runOnce(() -> m_llv.setLoadPipeline()),

                                                new TurnToAngle(m_drive, 180, true),

                                                new WaitCommand(.1),

                                                m_tf.followTrajectoryCommand(m_tf.getTrajFromCurrentlocation(endPose),
                                                                false),

                                                new WaitCommand(.25),

                                                new EjectPieceFromIntake(m_intake, 11).withTimeout(1));
                        }

                        if (autoselect1 == as1_balance_3) {

                                PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("Balance", 2.2, 7, false);

                                tempCommand = new SequentialCommandGroup(

                                                m_tf.followTrajectoryCommand(traj2, true).withTimeout(3),
                                                new AutoBalance(m_drive));

                        }

                }

                if (startLocation == sl_bumpShelf_3 && autoselect1 == as1_secondCube_2) {

                        HashMap<String, Command> eventMap1 = new HashMap<>();
                        eventMap1.put("DropIntake1",
                                        new GroundIntakeUprightConePositions(m_lift, m_wrist, m_extend, m_intake)
                                                        .withTimeout(5));
                        eventMap1.put("RunIntake1", new IntakePieceStopMotor(m_intake, 11).withTimeout(6));

                        HashMap<String, Command> eventMap2 = new HashMap<>();
                        eventMap2.put("HomePosition", new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist)
                                        .withTimeout(5));
                        eventMap2.put("Eject1", new EjectPieceFromIntake(m_intake, 12).withTimeout(1));

                        PathPlannerTrajectory eventPath1 = bumpEventPathsList.get(0);
                        PathPlannerTrajectory eventPath2 = bumpEventPathsList.get(1);

                        FollowPathWithEvents eventCommand1 = new FollowPathWithEvents(
                                        m_tf.followTrajectoryCommand(eventPath1, true),
                                        eventPath1.getMarkers(), eventMap1);
                        FollowPathWithEvents eventCommand2 = new FollowPathWithEvents(
                                        m_tf.followTrajectoryCommand(eventPath2, false),
                                        eventPath2.getMarkers(), eventMap2);

                        tempCommand = new SequentialCommandGroup(
                                        m_intake.tipRearCube(90),
                                        eventCommand1,
                                        new WaitCommand(0.5),
                                        m_intake.tipRearCube(0),
                                        eventCommand2,
                                        new EjectPieceFromIntake(m_intake, 12).withTimeout(1));

                }

                if (startLocation == sl_bumpShelf_3 && autoselect1 == as1_secondCubeAlt_4) {

                        if (runTraj) {
                                tempCommand = new SequentialCommandGroup(
                                                Commands.runOnce(() -> m_lift.setController(
                                                                LiftArmConstants.liftArmFastConstraints, 1, false)),
                                                m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(0), true),
                                                new WaitCommand(.1),
                                                // new TurnToGamepiece(m_drive, autoselect1, true),
                                                new GroundIntakeCubePositions(m_lift, m_wrist, m_extend, m_intake),
                                                new WaitCommand(.1),
                                                m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(1), false)
                                                                .raceWith(new IntakePieceStopMotor(m_intake, 11)),
                                                new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist),
                                                m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(2), false),
                                                new EjectPieceFromIntake(m_intake, 11).withTimeout(1));
                        } else {
                                
                                PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("BumpDriveOut", 2.75, 1,
                                                false);

                                endPose = fieldTagsBlue.BLUE_BUMP.getPose();

                                if (DriverStation.getAlliance() == Alliance.Red)

                                        endPose = fieldTagsRed.RED_BUMP.getPose();

                                // need to move x to be short of tag by ?? 1 meters

                                endPose.transformBy(new Transform2d(new Translation2d(-1, 0), new Rotation2d()));

                                tempCommand = new SequentialCommandGroup(

                                                Commands.runOnce(() -> m_lift.setController(
                                                                LiftArmConstants.liftArmFastConstraints, 1, false)),

                                                m_tf.followTrajectoryCommand(traj2, true).withTimeout(3),

                                                new WaitCommand(.1),

                                                new TurnToGamepiece(m_drive, -2, true),

                                                new GroundIntakeCubePositions(m_lift, m_wrist, m_extend, m_intake)
                                                                .withTimeout(5),

                                                new ParallelRaceGroup(

                                                                new SetSwerveDriveGamepiece(m_drive, m_llv, false,
                                                                                () -> -.5,
                                                                                () -> 0, () -> 0).withTimeout(5),
                                                                new IntakePieceStopMotor(m_intake,
                                                                                11).withTimeout(4)),

                                                new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist)
                                                                .withTimeout(4),

                                                Commands.runOnce(() -> m_llv.setLoadPipeline()),

                                                new TurnToAngle(m_drive, 180, true),

                                                new WaitCommand(.1),

                                                m_tf.followTrajectoryCommand(m_tf.getTrajFromCurrentlocation(endPose),
                                                                false),

                                                new WaitCommand(.25),

                                                new EjectPieceFromIntake(m_intake, 11).withTimeout(1));
                        }
                }

                if (startLocation == sl_bumpShelf_3 && autoselect1 == as_1_driveOutZone_5) {

                        PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("BumpDriveOut", 2.2, 4, false);

                        tempCommand = m_tf.followTrajectoryCommand(traj2, true).withTimeout(3);

                }

                if (startLocation == sl_noBumpShelf_2 && autoselect1 == as_1_driveOutZone_5) {

                        PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("NoBumpDriveOut", 2.2, 4, false);

                        tempCommand = m_tf.followTrajectoryCommand(traj2, true).withTimeout(3);

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

}
