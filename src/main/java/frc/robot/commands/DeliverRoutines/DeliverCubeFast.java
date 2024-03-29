// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.Auto.DoNothing;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverCubeFast extends SequentialCommandGroup {
        /** Creates a new DeliverCubeMid. */
        public DeliverCubeFast(LiftArmSubsystem lift, WristSubsystem wrist, IntakeSubsystem intake,
                        ExtendArmSubsystem extend,
                        boolean toplevel, double delShootSec, boolean quickEnd) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                double pullInVolts = 4;
                addCommands(

                                Commands.runOnce(() -> intake.setMotorVolts(.5)),

                                new ConditionalCommand(

                                                Commands.runOnce(() -> lift.setController(
                                                                LiftArmConstants.liftArmFastConstraints, 12, false)),

                                                Commands.runOnce(() -> lift.setController(
                                                                LiftArmConstants.liftArmFastConstraints, 6.5, false)),

                                                () -> toplevel),

                                new ConditionalCommand(

                                                Commands.runOnce(() -> wrist.setController(
                                                                WristConstants.wristFastConstraints, .5, false)),

                                                new DoNothing(),

                                                () -> toplevel),

                                Commands.runOnce(() -> intake.setMotorVolts(pullInVolts)),

                                new WaitCommand(0.5),

                                new WaitLiftAtTarget(lift, .5, 1, 2.5),

                                new WaitCommand(delShootSec),

                                new ConditionalCommand(

                                                new EjectPieceFromIntake(intake, 10).withTimeout(1),

                                                new EjectPieceFromIntake(intake, 5).withTimeout(1),

                                                () -> toplevel),

                                
                                new SetWristGoal(wrist, WristConstants.wristFastConstraints,
                                                presetWristAngles.HOME.getAngleRads()).asProxy(),

                                new WaitWristAtTarget(wrist, 1, .4, 1).asProxy(),

                                new SetLiftGoal(lift, presetLiftAngles.HOME.getInches()).asProxy(),

                                new ConditionalCommand(

                                                new WaitLiftAtTarget(lift, 1, 1, 1).asProxy(), new DoNothing(),
                                                () -> !quickEnd));

        }

}
