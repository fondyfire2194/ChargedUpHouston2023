// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PickupRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ExtendArmConstants;
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
public class LoadSubstationPositions extends SequentialCommandGroup {

        public LoadSubstationPositions(LiftArmSubsystem lift, WristSubsystem wrist, ExtendArmSubsystem extend,
                        IntakeSubsystem intake) {
                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand() extend, Intake);
                // assumes start from travel position

                addCommands(

                                new SetLiftGoal(lift, presetLiftAngles.PICKUP_SIDE_LOAD_STATION
                                                                .getInches()) .asProxy(),
                                                                                              
                                new SetWristGoal(wrist,
                                                presetWristAngles.PICKUP_SIDE_LOAD_STATION
                                                                .getAngleRads()) .asProxy(),                                            

                                new WaitLiftAtTarget(lift, 1, .5).asProxy(),

                                new WaitWristAtTarget(wrist, 1, .2).asProxy(),

                                new SetExtArmGoal(extend,
                                                ExtendArmConstants.extendArmFastConstraints,
                                                presetExtArmDistances.PICKUP_SIDE_LOAD_STATION
                                                                .getDistance()) .asProxy(),
                                               

                                new WaitExtendAtTarget(extend, 1, 4).asProxy());

        }
}