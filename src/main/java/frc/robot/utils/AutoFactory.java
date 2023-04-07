// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.DeliverRoutines.DeliverCubeFast;
import frc.robot.commands.DeliverRoutines.DeliverPiecePositions;
import frc.robot.commands.TeleopRoutines.DriveAndPickup;
import frc.robot.commands.TeleopRoutines.TurnToAngle;
import frc.robot.commands.TeleopRoutines.TurnToGamepiece;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

/** Add your docs here. */
public class AutoFactory {

    // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(null, null, null, null,
    // null, null, null);

    int sl_coopShelf_0 = 0;
    int sl_coopLeftPipe_1 = 1;
    int sl_coopRightPipe_2 = 2;
    int sl_leftShelf_3 = 3;
    int sl_rightShelf_4 = 4;

    int as_doNothing_0 = 0;
    int as_pushCube_1 = 1;
    int as_deliverMid_2 = 2;
    int as_deliverTop_3 = 3;

    int as1__balanceCharge_1 = 1;
    int as1_driveThruCharge_2 = 2;
    int as1_secondCube_3 = 3;

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

    private PathPlannerTrajectory traj2;

    private Command command1 = new DoNothing();

    private Command command0 = new DoNothing();

    private boolean traj1Reqd;

    private Command command2 = new DoNothing();

    private boolean traj2Reqd;

    private Command command3 = new DoNothing();

    public String traj1name = "PushCubeCenter";

    public String traj2name = "BackUpCenter";

    private double startTime;

    private boolean secondPieceRight;

    private boolean secondPieceLeft;

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

        m_startLocationChooser.setDefaultOption("CoopShelf", 0);

        m_startLocationChooser.addOption("CoopLeftPipe", 1);

        m_startLocationChooser.addOption("CoopRightPipe", 2);

        m_startLocationChooser.addOption("LeftShelf", 3);

        m_startLocationChooser.addOption("RightShelf", 4);

        m_autoChooser.setDefaultOption("Do Nothing", 0);

        m_autoChooser.addOption("PushCube", 1);

        m_autoChooser.addOption("DeliverMid", 2);

        m_autoChooser.addOption("DeliverTop", 3);

        m_autoChooser1.setDefaultOption("Do Nothing", 0);

        m_autoChooser1.addOption("BalanceCharge", 1);

        m_autoChooser1.addOption("DriveThroughCharge", 2);

        m_autoChooser1.addOption("PickupScore2ndCube", 3);

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

        if (startLocation == sl_leftShelf_3 && autoselect == as_pushCube_1
                && DriverStation.getAlliance() == Alliance.Blue) {
            traj1name = "PushCubeLeftShelf";
            traj1Reqd = true;
        }
        if (startLocation == sl_leftShelf_3 && autoselect == as_pushCube_1
                && DriverStation.getAlliance() == Alliance.Red) {
            traj1name = "PushCubeRightShelf";
            traj1Reqd = true;
        }

        if (startLocation == sl_rightShelf_4 && autoselect == as_pushCube_1
                && DriverStation.getAlliance() == Alliance.Blue) {
            traj1name = "PushCubeRightShelf";
            traj1Reqd = true;
        }

        if (startLocation == sl_rightShelf_4 && autoselect == as_pushCube_1
                && DriverStation.getAlliance() == Alliance.Red) {
            traj1name = "PushCubeLeftShelf";
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

        traj2Reqd = false;

        int startLocation = m_startLocationChooser.getSelected();

        int autoselect1 = m_autoChooser1.getSelected();

        if (startLocation <= sl_coopRightPipe_2) {// any of the coop starts
            if (autoselect1 == as1__balanceCharge_1) {
                tempCommand = m_drive.autoBalance();
            }

            if (startLocation == sl_coopShelf_0 && autoselect1 == as1_driveThruCharge_2) {
                traj2name = "BackUpCenter";
                traj2Reqd = true;
            }

            if (startLocation == sl_coopLeftPipe_1 && autoselect1 == as1_driveThruCharge_2
                    && DriverStation.getAlliance() == Alliance.Blue) {
                traj2name = "BackUpLeftCenter";
                traj2Reqd = true;
            }

            if (startLocation == sl_coopLeftPipe_1 && autoselect1 == as1_driveThruCharge_2
                    && DriverStation.getAlliance() == Alliance.Red) {
                traj2name = "BackUpRightCenter";
                traj2Reqd = true;
            }

            if (startLocation == sl_coopRightPipe_2 && autoselect1 == as1_driveThruCharge_2
                    && DriverStation.getAlliance() == Alliance.Blue) {
                traj2name = "BackUpRightCenter";
                traj2Reqd = true;
            }

            if (startLocation == sl_coopRightPipe_2 && autoselect1 == as1_driveThruCharge_2
                    && DriverStation.getAlliance() == Alliance.Red) {
                traj2name = "BackUpLeftCenter";
                traj2Reqd = true;
            }
        }

        if (startLocation == sl_leftShelf_3 && autoselect1 == as1_secondCube_3
                && DriverStation.getAlliance() == Alliance.Blue) {

                    // traj2name = "BackUpLeftShelf";         
            traj2name = "LeftShelfToLeftCubeRotate";
            traj2Reqd = true;
            secondPieceLeft = false;
            secondPieceRight = false;
        }

        if (startLocation == sl_leftShelf_3 && autoselect1 == as1_secondCube_3
                && DriverStation.getAlliance() == Alliance.Red) {
            traj2name = "BackUpRightShelf";
            traj2Reqd = true;
            secondPieceRight = true;
            secondPieceLeft = false;
        }

        if (startLocation == sl_rightShelf_4 && autoselect1 == as1_secondCube_3
                && DriverStation.getAlliance() == Alliance.Blue) {
            traj2name = "BackUpRightShelf";
            traj2Reqd = true;
            secondPieceRight = true;
            secondPieceLeft = false;
        }

        if (startLocation == sl_rightShelf_4 && autoselect1 == as1_secondCube_3
                && DriverStation.getAlliance() == Alliance.Red) {
            traj2name = "BackUpLeftShelf";
            traj2Reqd = true;
            secondPieceLeft = true;
            secondPieceRight = false;
        }

        if (traj2Reqd) {
            traj2 = m_tf.getPathPlannerTrajectory(traj2name, 2, 1, false);
            tempCommand = m_tf.followTrajectoryCommand(traj2, !traj1Reqd);
        }

        return tempCommand;
    }

    public Command getCommand3Left() {

        PathPlannerTrajectory traj = m_tf.getPathPlannerTrajectory("LeftCubetoLeftShelf", 2, 1, false);

        return new SequentialCommandGroup(

                new TurnToGamepiece(m_drive, -1, -30, true),

                //new DriveAndPickup(m_drive),

                new TurnToAngle(m_drive, 180, false),

                m_tf.followTrajectoryCommand(traj, false),

                new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false));

    }

    public Command getCommand3Right() {

        PathPlannerTrajectory traj = m_tf.getPathPlannerTrajectory("RightCubetoRightShelf", 2, 1, false);

        return new SequentialCommandGroup(

                new TurnToGamepiece(m_drive, 1, 30, true),

                new DriveAndPickup(m_drive),

                new TurnToAngle(m_drive, 180, false),

                m_tf.followTrajectoryCommand(traj, false),

                new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false));

    }

    public void createCommands() {

        command0 = new DoNothing();
        command1 = new DoNothing();
        command2 = new DoNothing();
        command3 = new DoNothing();

        command1 = getCommand1();
        command2 = getCommand2();
        if (secondPieceLeft)
            command3 = getCommand3Left();
        if (secondPieceRight)
            command3 = getCommand3Right();

    }

    public Command getAutonomousCommand() {

        startTime = Timer.getFPGATimestamp();

        createCommands();

        autonomousCommand = new SequentialCommandGroup(command0, command1, command2, command3);

        return autonomousCommand;

    }

    private Command getDeliverMid() {

        return new SequentialCommandGroup(

                new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false),

                new WaitCommand(.5)

        );

    }

    private Command getDeliverTop() {

        return new SequentialCommandGroup(

                new DeliverCubeFast(m_lift, m_wrist, m_intake, m_extend, false),

                new WaitCommand(.5)

        );

    }

}
