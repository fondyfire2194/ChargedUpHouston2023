// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeliverRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ExtendArmConstants;
import frc.robot.Constants.LiftArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.ExtendArm.SetExtArmGoal;
import frc.robot.commands.ExtendArm.WaitExtendAtTarget;
import frc.robot.commands.LiftArm.SetLiftGoal;
import frc.robot.commands.LiftArm.WaitLiftAtTarget;
import frc.robot.commands.Wrist.SetWristGoal;
import frc.robot.commands.Wrist.WaitWristAtTarget;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.ExtendArmSubsystem.presetExtArmDistances;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.LiftArmSubsystem.presetLiftAngles;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.presetWristAngles;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverPiecePositions extends SequentialCommandGroup {
        /** Creates a new DeliverSelectedPieceToSelectedTarget. */
        public DeliverPiecePositions(LiftArmSubsystem lift, ExtendArmSubsystem extend, WristSubsystem wrist,
                        IntakeSubsystem intake) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(

                                new SetLiftGoal(lift, LiftArmConstants.liftArmFastConstraints, lift.deliverInches)
                                                .asProxy(),

                                new SetWristGoal(wrist, WristConstants.wristFastConstraints, wrist.deliverAngleRads)
                                                .asProxy(),

                                new WaitLiftAtTarget(lift, .5, 3, 1).asProxy(),

                                new WaitWristAtTarget(wrist, .5, .2, 3).asProxy(),

                                new SetExtArmGoal(extend, ExtendArmConstants.extendArmFastConstraints,
                                                extend.deliverDistance)
                                                .asProxy(),

                                new WaitExtendAtTarget(extend, .5, 5, 3).asProxy(),

                                new EjectPieceFromIntake(intake, 10).withTimeout(1),

                                new SetExtArmGoal(extend, ExtendArmConstants.extendArmFastConstraints,
                                                presetExtArmDistances.TRAVEL.getDistance()).asProxy(),

                                new SetWristGoal(wrist, WristConstants.wristFastConstraints,
                                                presetWristAngles.TRAVEL.getAngleRads()).asProxy(),

                                new WaitWristAtTarget(wrist, 1, .4, 2).asProxy(),

                                new SetLiftGoal(lift, presetLiftAngles.TRAVEL.getInches()).asProxy(),

                                new WaitLiftAtTarget(lift, 1, 1, 1).asProxy());

        }
}
