// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulation.SimConstants;

/** Add your docs here. */
public class GameHandlerSubsystem extends SubsystemBase {

        public static int gridHeight = 0;// 0 floor, 1 mid, 2 high

        /**
         * Grid values as viewed by Blue Drivers starting on their right at the field
         * RIGHT
         * Red drivers have the field RIGHT on their left
         * The X distance is the center of the robot as it strafes past the grids
         * For Blue it will be the distance used.
         * For Red it will be the field length minus that distance
         * Similarly Y will be used direct or by subtracting from the field width
         * Grids are 66" wide
         */

        public enum GridDrop {

                LEFT_CUBE(SimConstants.Tags.aprilTagsRed[0], false,
                                SimConstants.Tags.aprilTagsBlue[5],
                                false),

                LEFT_PIPE(SimConstants.leftPipeRed, true,
                                SimConstants.leftPipeBlue,
                                true),

                COOP_LEFT_PIPE(SimConstants.coopleftPipeRed, true,
                                SimConstants.coopleftPipeBlue, true),

                COOP_CUBE(SimConstants.Tags.aprilTagsRed[1], false,
                                SimConstants.Tags.aprilTagsBlue[6], false),

                COOP_RIGHT_PIPE(SimConstants.cooprightPipeRed, true,
                                SimConstants.cooprightPipeBlue,
                                true),

                RIGHT_PIPE(SimConstants.rightPipeRed, true,
                                SimConstants.rightPipeBlue,
                                true),

                RIGHT_CUBE(SimConstants.Tags.aprilTagsRed[2], false,
                                SimConstants.Tags.aprilTagsBlue[7],
                                false);

                private final Pose2d bluePose;

                private final Pose2d redPose;

                private final boolean isPipeRed;

                private final boolean isPipeBlue;

                private final static int length = GridDrop.values().length;

                private GridDrop(Pose2d redPose, boolean isPipeRed, Pose2d bluePose,
                                boolean isPipeBlue) {
                        this.redPose = redPose;
                        this.bluePose = bluePose;
                        this.isPipeRed = isPipeRed;

                        this.isPipeBlue = isPipeBlue;

                }

                public Pose2d getBluePose() {
                        return bluePose;
                }

                public Pose2d getRedPose() {
                        return redPose;
                }

                public boolean getIsPipeRed() {
                        return isPipeRed;
                }

                public boolean getIsPipeBlue() {
                        return isPipeBlue;
                }
        }

        public enum dropOffLevel {

                GROUND_LEVEL(0),
                MID_LEVEL(1),
                TOP_LEVEL(2);

                private int level;

                private dropOffLevel(int level) {
                        this.level = level;
                }
        }

        public enum gamePiece {

                CUBE(0),

                CONE(1);

                private int type;

                private gamePiece(int type) {
                        this.type = type;
                }
        }

        public enum robotPiece {

                CUBE(0),
                CONE(1),
                NO_PIECE(2);

                private int type;

                private robotPiece(int type) {
                        this.type = type;
                }
        }

        public enum fieldTagsBlue {

                RED_LEFT(1, SimConstants.Tags.aprilTagsBlue[0]),
                RED_CENTER(2, SimConstants.Tags.aprilTagsBlue[1]),
                RED_RIGHT(3, SimConstants.Tags.aprilTagsBlue[2]),
                BLUE_LOAD(4, SimConstants.Tags.aprilTagsBlue[3]),
                RED_LOAD(5, SimConstants.Tags.aprilTagsBlue[5]),
                BLUE_LEFT(6, SimConstants.Tags.aprilTagsBlue[5]),
                BLUE_CENTER(7, SimConstants.Tags.aprilTagsBlue[6]),
                BLUE_RIGHT(8, SimConstants.Tags.aprilTagsBlue[7]);

                private int id;

                private Pose2d pose;

                private fieldTagsBlue(int id, Pose2d pose) {
                        this.id = id;
                        this.pose = pose;
                }

                public Pose2d getPose() {
                        return pose;
                }

                public int getID() {
                        return id;
                }

        }

        public enum fieldTagsRed {

                RED_LEFT(1, SimConstants.Tags.aprilTagsRed[0]),
                RED_CENTER(2, SimConstants.Tags.aprilTagsRed[1]),
                RED_RIGHT(3, SimConstants.Tags.aprilTagsRed[2]),
                BLUE_LOAD(4, SimConstants.Tags.aprilTagsRed[3]),
                RED_LOAD(5, SimConstants.Tags.aprilTagsRed[4]),
                BLUE_LEFT(6, SimConstants.Tags.aprilTagsRed[5]),
                BLUE_CENTER(7, SimConstants.Tags.aprilTagsRed[6]),
                BLUE_RIGHT(8, SimConstants.Tags.aprilTagsRed[7]);

                private int id;

                private Pose2d pose;

                private fieldTagsRed(int id, Pose2d pose) {
                        this.id = id;
                        this.pose = pose;
                }

                public Pose2d getPose() {
                        return pose;
                }

        }

        public dropOffLevel chosenLevel = dropOffLevel.MID_LEVEL;

        public gamePiece gamePieceType = gamePiece.CUBE;

        public double strafeDistance = 1.5;// meters

        public GameHandlerSubsystem() {

        }

     
        public boolean getAllianceBlue() {
                return (DriverStation.getAlliance() == Alliance.Blue);
        }

        public void setGamePieceType(gamePiece type) {
                gamePieceType = type;
        }

        public void toggleGamePieceType() {

                if (gamePieceType == gamePiece.CONE)

                        gamePieceType = gamePiece.CUBE;
                else
                        gamePieceType = gamePiece.CONE;
        }

        public gamePiece getGamePiecetype() {
                return gamePieceType;
        }

        @Override

        public void periodic() {

                // if (originalAlliance != DriverStation.getAlliance()) {
                // setActiveDropByNumber(dropNumberSet);
                // }

        }

}
