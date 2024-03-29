// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.PickupRoutines.GroundIntakeCubePositions;
import frc.robot.commands.PickupRoutines.IntakePieceStopMotor;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLiftTravel;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;

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
        int as1_secondCubeAlt_2 = 2;
        int as1_balance_3 = 3;
        int as_1_driveOutZone_4 = 4;

        List<PathPlannerTrajectory> pathGroup;

        // public Command autonomousCommand = new DoNothing();

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

        public List<PathPlannerTrajectory> noBumpStartTrajsAlt;

        private List<PathPlannerTrajectory> bumpStartTrajsAlt;

        List<PathPlannerTrajectory> balanceCommandList;

        List<PathPlannerTrajectory> bumpEventPathsList;

        public Pose2d endPose = new Pose2d();

        PathPlannerTrajectory t1;// = noBumpStartTrajsAlt.get(1);

        Pose2d pickup = new Pose2d(6.62, 4.78, new Rotation2d(Units.degreesToRadians(-21)));

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

                m_autoChooser1.addOption("DriveOutZone", 4);

                // Load trajectories to save time later

                noBumpStartTrajsAlt = m_tf.getPathPlannerTrajectoryGroup("NoBumpSideAlt",
                                1.5, 2., false);

                t1 = noBumpStartTrajsAlt.get(1);

                bumpStartTrajsAlt = m_tf.getPathPlannerTrajectoryGroup("BumpSideAlt",
                                1.5, 2.0, false);

                balanceCommandList = m_tf.getPathPlannerTrajectoryGroup("BackUpCenter", 2.2,
                                2, false);

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

                if ((startLocation == sl_coopShelf_0 || startLocation == sl_noBumpShelf_2
                                || startLocation == sl_bumpShelf_3)
                                && autoselect == as_deliverMid_2) {
                        tempCommand = new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false, 0,
                                        startLocation != sl_coopShelf_0);
                }

                if ((startLocation == sl_coopShelf_0 || startLocation == sl_noBumpShelf_2
                                || startLocation == sl_bumpShelf_3)
                                && autoselect == as_deliverTop_3) {
                        tempCommand = new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, true, 0,
                                        startLocation != sl_coopShelf_0);
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

                                tempCommand = new SequentialCommandGroup(

                                                m_tf.followTrajectoryCommand(balanceCommandList.get(0), true),
                                                new WaitCommand(1.5),
                                                m_tf.followTrajectoryCommand(balanceCommandList.get(1), false),
                                                new AutoBalanceBackwards(m_drive));
                        }

                }

                /**
                 * Blue drive station no bump shelf is tag Id 6
                 * Red drive station no bump shelf is tag Id 3
                 * 
                 */

                if (startLocation == sl_noBumpShelf_2 && autoselect1 == as1_secondCubeAlt_2) {

                        m_llv.setActiveCamera(0);// front camera

                        m_llv.setRedNoBumpPipeline();// look for AprilTag 3

                        if (DriverStation.getAlliance() == Alliance.Blue)

                                m_llv.setBlueNoBumpPipeline();// look for April Tag 6

                        m_drive.setAllowVisionCorrection(true);// allow vision correction

                        tempCommand = new SequentialCommandGroup(

                                        Commands.runOnce(() -> m_lift.setController(

                                                        LiftArmConstants.liftArmFastConstraints, 1, false)),

                                        Commands.runOnce(() -> m_drive.setAllowVisionCorrection(true)), // inhibit
                                                                                                         // vision
                                                                                                         // correction

                                        // move and rotate

                                        m_tf.followTrajectoryCommand(noBumpStartTrajsAlt.get(0), true),

                                        Commands.runOnce(() -> m_llv.setActiveCamera(1)), // rear camera

                                        Commands.runOnce(() -> m_drive.setAllowVisionCorrection(false)), // inhibit
                                                                                                         // vision
                                                                                                         // correction

                                        // new GroundIntakeCubePositions(m_lift, m_wrist,
                                        // m_extend, m_intake)
                                        // .withTimeout(1),

                                        new WaitCommand(1),

                                        Commands.runOnce(() -> m_drive.setAllowVisionCorrection(false)), // inhibit
                                                                                                         // vision
                                                                                                         // correction

                                        Commands.runOnce(() -> t1 = m_tf
                                                        .getTrajFromCurrentlocation(pickup)),

                                        new ParallelCommandGroup(

                                                        new IntakePieceStopMotor(m_intake, 11),

                                                        m_tf.followTrajectoryCommand(t1,
                                                                        false)),

                                        new StopIntake(m_intake),

                                        new WaitCommand(.1),

                                        new SetLiftGoal(m_lift, 2),

                                        new WaitCommand(.5),

                                        new ParallelCommandGroup(

                                                        new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist)
                                                                        .withTimeout(2),

                                                        m_tf.followTrajectoryCommand(noBumpStartTrajsAlt.get(2),
                                                                        false)),

                                        Commands.runOnce(() -> m_llv.setActiveCamera(0)), // FRONT camera

                                        Commands.runOnce(() -> m_drive.setAllowVisionCorrection(false)),

                                        new WaitCommand(.1),

                                        Commands.runOnce(() -> m_drive.setAllowVisionCorrection(false)),

                                        m_tf.followTrajectoryCommand(noBumpStartTrajsAlt.get(3),
                                                        false),

                                        new EjectPieceFromIntake(m_intake, 9).withTimeout(1));

                }

                if (autoselect1 == as1_balance_3)

                {

                        PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("Balance", 2.2, 2, false);

                        tempCommand = new SequentialCommandGroup(

                                        m_tf.followTrajectoryCommand(traj2, true).withTimeout(3),

                                        new AutoBalance(m_drive));

                }

                if (startLocation == sl_bumpShelf_3 && autoselect1 == as1_secondCubeAlt_2) {

                        tempCommand = new SequentialCommandGroup(

                                        // Commands.runOnce(() -> m_llv.setCubeDetectorPipeline()),

                                        Commands.runOnce(() -> m_lift.setController(

                                                        LiftArmConstants.liftArmFastConstraints, 1, false)),

                                        // new ParallelCommandGroup(

                                        // move and rotate

                                        m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(0), true),

                                        // new SequentialCommandGroup(

                                        // new WaitCommand(2.5),

                                        new GroundIntakeCubePositions(m_lift, m_wrist,
                                                        m_extend, m_intake)
                                                        .withTimeout(1),

                                        new WaitCommand(.1),

                                        new ParallelCommandGroup(
                                                        new IntakePieceStopMotor(m_intake, 11),

                                                        m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(1),
                                                                        false)),

                                        // Commands.runOnce(() -> m_llv.setCubeDetectorPipeline()),

                                        // new ConditionalCommand(

                                        // new TrajectoryCorrectForCube(m_drive, m_llv),
                                        // new DoNothing(),
                                        // () -> RobotBase.isReal())),

                                        new StopIntake(m_intake),

                                        new WaitCommand(.1),

                                        new SetLiftGoal(m_lift, 2),

                                        new WaitCommand(.5),

                                        new ParallelCommandGroup(

                                                        new RetractWristExtendLiftTravel(m_lift, m_extend, m_wrist)
                                                                        .withTimeout(2),

                                                        m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(2),
                                                                        false)),

                                        new WaitCommand(.1),

                                        m_tf.followTrajectoryCommand(bumpStartTrajsAlt.get(3),
                                                        false),

                                        new EjectPieceFromIntake(m_intake, 9).withTimeout(1));
                }

                if (startLocation == sl_bumpShelf_3 && autoselect1 == as_1_driveOutZone_4) {

                        PathPlannerTrajectory traj2 = m_tf.getPathPlannerTrajectory("BumpDriveOut", 2.2, 4, false);

                        tempCommand = m_tf.followTrajectoryCommand(traj2, true).withTimeout(3);

                }

                if (startLocation == sl_noBumpShelf_2 && autoselect1 == as_1_driveOutZone_4) {

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

                m_drive.autonomousCommand = new SequentialCommandGroup(command0, command1, command2);

                return m_drive.autonomousCommand;

        }

}
