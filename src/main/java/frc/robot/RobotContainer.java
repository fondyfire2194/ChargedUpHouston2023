
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.DeliverRoutines.DeliverCubeFast;
import frc.robot.commands.DeliverRoutines.EjectPieceFromIntake;
import frc.robot.commands.DeliverRoutines.GetDeliverAngleSettings;
import frc.robot.commands.ExtendArm.JogExtendArm;
import frc.robot.commands.ExtendArm.PositionProfileExtendArm;
import frc.robot.commands.LiftArm.JogLiftArm;
import frc.robot.commands.LiftArm.PositionProfileLiftInches;
import frc.robot.commands.NTs.MonitorThreadExt;
import frc.robot.commands.NTs.MonitorThreadIntake;
import frc.robot.commands.NTs.MonitorThreadLift;
import frc.robot.commands.NTs.MonitorThreadWrist;
import frc.robot.commands.PickupRoutines.GroundIntakeCubePositions;
import frc.robot.commands.PickupRoutines.GroundIntakeUprightConePositions;
import frc.robot.commands.PickupRoutines.IntakePiece;
import frc.robot.commands.PickupRoutines.LoadStationPositions;
import frc.robot.commands.PickupRoutines.LoadSubstationPositions;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLiftHome;
import frc.robot.commands.TeleopRoutines.RetractWristExtendLiftTravel;
import frc.robot.commands.TeleopRoutines.RotateToAngle;
import frc.robot.commands.Wrist.JogWrist;
import frc.robot.commands.Wrist.PositionProfileWrist;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.SetSwerveDriveSlow;
import frc.robot.commands.swerve.ToggleFieldOriented;
import frc.robot.oi.ShuffleboardArms;
import frc.robot.oi.ShuffleboardCompetition;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LLDriveLinkerSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LightStrip;
import frc.robot.subsystems.LightStrip.ledColors;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.AutoFactory;
import frc.robot.utils.TrajectoryFactory;

public class RobotContainer {

        // The robot's subsystems
        final DriveSubsystem m_drive;

        final LiftArmSubsystem m_liftArm;// = new LiftArmSubsystem();

        final ExtendArmSubsystem m_extendArm;// = new ExtendArmSubsystem();

        final IntakeSubsystem m_intake;

        final WristSubsystem m_wrist = new WristSubsystem();

        final LimelightVision m_llv;// = new LimelightVision();

        final ShuffleboardCompetition m_shc;

        final ShuffleboardArms m_sharm;

        public AutoFactory m_autoFactory;

        public TrajectoryFactory m_tf;

        public FieldSim m_fieldSim = null;

        // The driver, codriver and arm controllers

        public CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);

        public CommandXboxController m_coDriverController = new CommandXboxController(
                        OIConstants.kCoDriverControllerPort);

        public CommandXboxController m_armsController = new CommandXboxController(
                        OIConstants.kArmControllerPort);

        // final PowerDistribution m_pdp = new PowerDistribution();

        public LimelightVision m_llvis = new LimelightVision();

        public LLDriveLinkerSubsystem m_lldv;

        public LightStrip m_ls = new LightStrip(9, 36);

        public MonitorThreadExt mext;
        public MonitorThreadLift mlift;
        public MonitorThreadWrist mwrist;
        public MonitorThreadIntake mIntake;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                m_drive = new DriveSubsystem();

                Pref.deleteUnused();

                Pref.addMissing();

                m_llv = new LimelightVision();

                m_ls.setColor(ledColors.PURPLE);

                m_lldv = new LLDriveLinkerSubsystem(m_llv, m_drive);

                m_liftArm = new LiftArmSubsystem();

                mlift = new MonitorThreadLift(m_liftArm);

                mlift.startThread();

                mwrist = new MonitorThreadWrist(m_wrist);

                mwrist.startThread();

                SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

                LiveWindow.disableAllTelemetry();

                m_intake = new IntakeSubsystem();

                mIntake = new MonitorThreadIntake(m_intake);

                mIntake.startThread();

                m_extendArm = new ExtendArmSubsystem();

                mext = new MonitorThreadExt(m_extendArm);

                mext.startThread();

                m_tf = new TrajectoryFactory(m_drive, m_fieldSim);

                m_autoFactory = new AutoFactory(m_drive, m_liftArm, m_extendArm, m_wrist, m_intake, m_tf);

                m_fieldSim = new FieldSim(m_drive);

                m_fieldSim.initSim();

                m_shc = new ShuffleboardCompetition(m_llv, m_drive, m_autoFactory,
                                m_liftArm, m_extendArm,
                                m_wrist, m_intake);

                m_sharm = new ShuffleboardArms(m_liftArm, m_extendArm, m_wrist,
                                m_intake, m_tf);

                setDefaultCommands();

                configDriverButtons();

                configCodriverButtons();

                configArmsButtons();

        }

        private void setDefaultCommands() {

                m_drive.setDefaultCommand(getDriveCommand());

                m_extendArm.setDefaultCommand(new PositionProfileExtendArm(m_extendArm,
                                m_liftArm));

                m_liftArm.setDefaultCommand(new PositionProfileLiftInches(m_liftArm));

                m_wrist.setDefaultCommand(new PositionProfileWrist(m_wrist, m_liftArm));

        }

        void configDriverButtons() {

                m_driverController.leftTrigger()
                                .whileTrue(getSlowDriveCommand());

                // DO NOT USE LeftBumper m_driverController.leftBumper().

                m_driverController.rightTrigger().whileTrue(new IntakePiece(m_intake, 11));

                m_driverController.rightBumper().whileTrue(new EjectPieceFromIntake(m_intake, 12));

                m_driverController.b().onTrue(new ConditionalCommand(deliverPositionsCommand(1).withTimeout(12),
                                deliverPositionsCommand(2).withTimeout(15),
                                () -> m_driverController.getHID().getLeftBumper()));

                m_driverController.x().onTrue(new ConditionalCommand(new ToggleFieldOriented(m_drive),
                                Commands.runOnce(() -> m_drive.resetGyro()),
                                () -> m_driverController.getHID().getLeftBumper()));

                m_driverController.y()
                                .onTrue(new LoadStationPositions(m_liftArm, m_wrist, m_extendArm, m_intake)
                                                .withTimeout(10))
                                .onTrue(new IntakePiece(m_intake, 11));

                // m_driverController.a()
                // .onTrue(new ToggleFieldOriented(m_drive));

                m_driverController.back()
                                .onTrue(new InstantCommand(() -> m_drive.resetGyro()));

                m_driverController.start()
                                .onTrue(new ConditionalCommand(
                                                new RetractWristExtendLiftHome(m_liftArm, m_extendArm, m_wrist),
                                                new RetractWristExtendLiftTravel(m_liftArm, m_extendArm, m_wrist)
                                                                .withTimeout(8),

                                                () -> m_driverController.getHID().getLeftBumper()));

                // m_driverController.back()

                m_driverController.povUp().onTrue(new ConditionalCommand(Commands.runOnce(() -> m_liftArm.incGoal(.5)),
                                Commands.runOnce(() -> m_liftArm.incGoal(.25)),
                                () -> m_driverController.getHID().getLeftBumper()));

                m_driverController.povDown()
                                .onTrue(new ConditionalCommand(Commands.runOnce(() -> m_liftArm.incGoal(-.5)),
                                                Commands.runOnce(() -> m_liftArm.incGoal(-.25)),
                                                () -> m_driverController.getHID().getLeftBumper()));

                m_driverController.povLeft()
                                .onTrue(new ConditionalCommand(Commands.runOnce(() -> m_extendArm.incGoal(.75)),
                                                Commands.runOnce(() -> m_extendArm.incGoal(.25)),
                                                () -> m_driverController.getHID().getLeftBumper()));
                m_driverController.povRight()
                                .onTrue(new ConditionalCommand(Commands.runOnce(() -> m_extendArm.incGoal(-.75)),
                                                Commands.runOnce(() -> m_extendArm.incGoal(-.25)),
                                                () -> m_driverController.getHID().getLeftBumper()));

        }

        private void configCodriverButtons() {

                // DO NOT USE m_coDriverController.leftBumper()

                // m_coDriverController.leftTrigger()

                m_coDriverController.rightBumper()
                                .onTrue(new LoadSubstationPositions(m_liftArm, m_wrist, m_extendArm, m_intake)
                                                .withTimeout(8))
                                .onTrue(new IntakePiece(m_intake, 11));

                m_coDriverController.rightTrigger()
                                .onTrue(new ConditionalCommand(
                                                new RetractWristExtendLiftHome(m_liftArm, m_extendArm, m_wrist)
                                                                .withTimeout(8),
                                                new RetractWristExtendLiftTravel(m_liftArm, m_extendArm, m_wrist)
                                                                .withTimeout(8),
                                                () -> m_coDriverController.getHID().getLeftBumper()));

                m_coDriverController.a().onTrue(new ConditionalCommand(
                                new GroundIntakeUprightConePositions(m_liftArm, m_wrist, m_extendArm, m_intake)
                                                .withTimeout(10),
                                new GroundIntakeCubePositions(m_liftArm, m_wrist, m_extendArm, m_intake)
                                                .withTimeout(10),
                                () -> m_coDriverController.getHID().getLeftBumper()));

                m_coDriverController.x().onTrue(deliverPositionsCommand(1).withTimeout(8));

                // m_coDriverController.y()

                m_coDriverController.b().onTrue(new ConditionalCommand(deliverPositionsCommand(2).withTimeout(8),
                                deliverPositionsCommand(1).withTimeout(10),
                                () -> m_coDriverController.getHID().getLeftBumper()));

                m_coDriverController.start()
                                .onTrue(Commands.runOnce(() -> m_ls.togglePY()));

                m_coDriverController.povUp().onTrue(
                                new ConditionalCommand(Commands.runOnce(() -> m_wrist.incGoal(-.5)),
                                                Commands.runOnce(() -> m_wrist.incGoal(-.25)),
                                                () -> m_coDriverController.getHID().getLeftBumper()));

                m_coDriverController.povDown()
                                .onTrue(new ConditionalCommand(Commands.runOnce(() -> m_wrist.incGoal(.5)),
                                                Commands.runOnce(() -> m_wrist.incGoal(.25)),
                                                () -> m_coDriverController.getHID().getLeftBumper()));

                m_coDriverController.povLeft()
                                .onTrue(new ConditionalCommand(Commands.runOnce(() -> m_extendArm.incGoal(.75)),
                                                Commands.runOnce(() -> m_extendArm.incGoal(.25)),
                                                () -> m_coDriverController.getHID().getLeftBumper()));

                m_coDriverController.povRight()
                                .onTrue(new ConditionalCommand(Commands.runOnce(() -> m_extendArm.incGoal(-.75)),
                                                Commands.runOnce(() -> m_extendArm.incGoal(-.25)),
                                                () -> m_coDriverController.getHID().getLeftBumper()));

        }

        private void configArmsButtons() {

                m_armsController.leftBumper().whileTrue(getJogLiftArmCommand(m_armsController))
                                .onFalse(Commands.runOnce(() -> m_liftArm.setControllerAtPosition(),
                                                m_liftArm));

                m_armsController.leftTrigger().whileTrue(getJogExtendArmCommand(m_armsController))
                                .onFalse(Commands.runOnce(() -> m_extendArm.setControllerAtPosition(),
                                                m_extendArm));

                m_armsController.rightBumper().whileTrue(getJogWristCommand(m_armsController))
                                .onFalse(Commands.runOnce(() -> m_wrist.setControllerAtPosition(),
                                                m_wrist));

                m_armsController.a().onTrue(Commands.runOnce(
                                () -> m_wrist.setController(WristConstants.wristFastConstraints, 2, false)));

                m_armsController.x().onTrue(new DeliverCubeFast(m_liftArm, m_wrist, m_intake, m_extendArm, false));

                m_armsController.b().onTrue(new DeliverCubeFast(m_liftArm, m_wrist, m_intake, m_extendArm, true));

                m_armsController.y().onTrue(Commands.runOnce(
                                () -> m_liftArm.setController(LiftArmConstants.liftArmFastConstraints, 12, false)));

                // wrist

                m_armsController.start().onTrue(Commands.runOnce(
                                () -> m_wrist.setController(WristConstants.wristFastConstraints, .15, false)));

                m_armsController.povUp().onTrue(new RotateToAngle(m_drive, 178));
                // ext arm

                m_armsController.povDown().onTrue(new RotateToAngle(m_drive, 0));

                m_armsController.povLeft().onTrue(new RotateToAngle(m_drive, -45));

                m_armsController.povRight().onTrue(new RotateToAngle(m_drive, 45));

                m_armsController.rightTrigger()
                                .onTrue(Commands.runOnce(() -> m_extendArm.setController(
                                                ExtendArmConstants.extendArmFastConstraints, 0,
                                                false)));

                // m_armsController.back() DO NOT ASSIGN ALREADY USED IN JOG COMMANDS TO
                // OVERRIDE SOFTWARE LIMITS

        }

        public Command getDriveCommand() {
                return new SetSwerveDrive(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getSlowDriveCommand() {
                return new SetSwerveDriveSlow(m_drive,
                                () -> m_driverController.getRawAxis(1),
                                () -> m_driverController.getRawAxis(0),
                                () -> m_driverController.getRawAxis(4));

        }

        public Command getJogLiftArmCommand(CommandXboxController m_armController2) {
                return new JogLiftArm(m_liftArm, () -> -m_armsController.getRawAxis(1), m_armController2);
        }

        public Command getJogWristCommand(CommandXboxController m_armController2) {
                return new JogWrist(m_wrist, () -> m_armsController.getRawAxis(5), m_armController2);
        }

        public Command getStopDriveCommand() {
                return new InstantCommand(() -> m_drive.stopModules());
        }

        public Command getJogExtendArmCommand(CommandXboxController m_armController2) {
                return new JogExtendArm(m_extendArm, () -> -m_armsController.getRawAxis(0), m_armController2);
        }

        public Command deliverPositionsCommand(int level) {
                return new GetDeliverAngleSettings(m_liftArm, m_extendArm, m_wrist, m_intake,
                                level);
        }

        public Command getPositionExtendCommand() {
                return Commands.runOnce(() -> m_extendArm.setController((ExtendArmConstants.extendArmFastConstraints),
                                m_extendArm.getNextTarget(), false));
        }

        public Command getMoveLiftCommand() {

                return Commands.runOnce(() -> m_liftArm.setController(LiftArmConstants.liftArmFastConstraints,
                                m_liftArm.deliverInches, false))
                                .andThen(Commands.runOnce(
                                                () -> m_wrist.setController(WristConstants.wristFastConstraints,
                                                                m_wrist.deliverAngleRads, false)));
        }

        public Command getLoadSettings() {

                return new LoadSubstationPositions(m_liftArm, m_wrist, m_extendArm, m_intake);

        }

        public void simulationPeriodic() {

                m_fieldSim.periodic();
        }

        public void periodic() {
                m_fieldSim.periodic();
                // m_pt.update();

        }

}