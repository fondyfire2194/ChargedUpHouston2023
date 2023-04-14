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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverPiecePositionsTeleop extends SequentialCommandGroup {
        /** Creates a new DeliverSelectedPieceToSelectedTarget. */
        public DeliverPiecePositionsTeleop(LiftArmSubsystem lift, ExtendArmSubsystem extend, WristSubsystem wrist,
                        IntakeSubsystem intake) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                addCommands(

                                new SetLiftGoal(lift, LiftArmConstants.liftArmFastConstraints,
                                                lift.deliverInches).asProxy(),

                                new SetWristGoal(wrist, WristConstants.wristFastConstraints,
                                                wrist.deliverAngleRads).asProxy(),

                                new WaitLiftAtTarget(lift, .5, 3, 1).asProxy(),

                                new WaitWristAtTarget(wrist, .3, .2, 1).asProxy(),

                                new SetExtArmGoal(extend, ExtendArmConstants.extendArmFastConstraints,
                                                extend.deliverDistance).asProxy(),

                                new WaitExtendAtTarget(extend, .5, 5, 3).asProxy());

        }
}
