// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.LimelightVision.pipelines;
import frc.robot.subsystems.LimelightVision.pipelinetype;

public class LLDriveLinkerSubsystem extends SubsystemBase {
  /**
   * Creates a new LLD
   */
  private LimelightVision m_llv;
  private DriveSubsystem m_drive;
  private LimelightResults llresults;
  private boolean limelightExists;
  private int loopctr;
  private double neuralClassID;
  private double coneID = 0;
  private double cubeID = 1;

  public int activeLimelight;

  NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");

  IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();
  private int front = 0;
  private int rear = 1;
  private String[] m_names;

  public LLDriveLinkerSubsystem(LimelightVision llv, String[] names, DriveSubsystem drive) {
    m_llv = llv;
    m_drive = drive;
    m_names = names;
    activeLimelight = front;
  }

  @Override
  public void periodic() {
    loopctr++;

    if (RobotBase.isReal() && loopctr > 2 && m_drive.allowVision) {

      loopctr = 0;

      llresults = LimelightHelpers.getLatestResults(m_llv.activeName);

      if (m_llv.getCurrentPipelineType() == pipelinetype.fiducialmarkers && LimelightHelpers.getTV(m_llv.activeName)) {

        m_drive.botPose = getBotPose();
        m_drive.numberTags = llresults.targetingResults.targets_Fiducials.length;
        m_drive.hasTag = m_drive.numberTags > 0;
        m_drive.fiducialID = (int) LimelightHelpers.getFiducialID(m_llv.activeName);

        m_drive.latencyCaptureMs = LimelightHelpers.getLatency_Capture(m_llv.activeName);
        m_drive.latencyCaptureMs /= 1000;
        m_drive.latencyPipelineMs = LimelightHelpers.getLatency_Pipeline(m_llv.activeName);
        m_drive.latencyPipelineMs /= 1000;

        if (m_drive.numberTags >= 1) {
          m_drive.tagid1 = (int) llresults.targetingResults.targets_Fiducials[0].fiducialID;
          m_drive.tx = LimelightHelpers.getTX(m_llv.activeName);
        }
        m_drive.tagid2 = -1;
        if (m_drive.numberTags >= 2)
          m_drive.tagid2 = (int) llresults.targetingResults.targets_Fiducials[1].fiducialID;

      }

      if (m_llv.getCurrentPipelineType() == pipelinetype.retroreflective) {

        m_drive.numberTargets = llresults.targetingResults.targets_Retro.length;
        m_drive.hasTarget = m_drive.numberTargets > 0;
        m_drive.tx = LimelightHelpers.getTX(m_llv.activeName);
        m_drive.ty = LimelightHelpers.getTY(m_llv.activeName);
        m_drive.targetArea = LimelightHelpers.getTA(m_llv.activeName);

      }

      if (m_llv.getCurrentPipeline() == pipelines.CONE_DETECT) {

        neuralClassID = LimelightHelpers.getNeuralClassID(m_llv.activeName);

        boolean temp = false;

        if (neuralClassID == coneID)
          temp = true;

        m_drive.coneFound = temp;

        // String temp = LimelightHelpers.getNeuralClassName(m_llv.activeName);
        // SmartDashboard.putString("NCN1", temp);
        m_drive.tx = LimelightHelpers.getTX(m_llv.activeName);
        m_drive.ty = LimelightHelpers.getTY(m_llv.activeName);
        m_drive.targetArea = LimelightHelpers.getTA(m_llv.activeName);
      } else {
        m_drive.coneFound = false;
      }

      if (m_llv.getCurrentPipeline() == pipelines.CUBE_DETECT) {

        neuralClassID = LimelightHelpers.getNeuralClassID(m_llv.activeName);

        boolean temp = false;

        if (neuralClassID == cubeID)
          temp = true;

        m_drive.cubeFound = temp;

        m_drive.tx = LimelightHelpers.getTX(m_llv.activeName);
        m_drive.ty = LimelightHelpers.getTY(m_llv.activeName);
        m_drive.targetArea = LimelightHelpers.getTA(m_llv.activeName);
      } else {
        m_drive.cubeFound = false;
      }

    }

    Pose3d robotPose3d = new Pose3d(m_drive.getEstimatedPosition());
    robotPose3d = robotPose3d
        .exp(
            new Twist3d(
                0.0,
                0.0,
                Math.abs(Units.degreesToRadians(m_drive.getCompedGyroPitch())) * DriveConstants.kTrackWidth / 2.0,
                0.0,
                Units.degreesToRadians(m_drive.getCompedGyroPitch()),
                0.0))
        .exp(
            new Twist3d(
                0.0,
                0.0,
                Math.abs(Units.degreesToRadians(m_drive.getGyroRoll())) * DriveConstants.kWheelBase / 2.0,
                Units.degreesToRadians(m_drive.getGyroRoll()),
                0.0,
                0.0));

    tagsTable
        .getEntry("pose_robot")
        .setDoubleArray(
            new double[] {
                robotPose3d.getTranslation().getX(),
                robotPose3d.getTranslation().getY(),
                robotPose3d.getTranslation().getZ(),
                robotPose3d.getRotation().getQuaternion().getW(),
                robotPose3d.getRotation().getQuaternion().getX(),
                robotPose3d.getRotation().getQuaternion().getY(),
                robotPose3d.getRotation().getQuaternion().getZ()
            });

    Pose3d pose = getRobotPose_FieldSpace();

    tagsTable
        .getEntry("pose_" + m_drive.tagid1)
        .setDoubleArray(
            new double[] {
                pose.getTranslation().getX(),
                pose.getTranslation().getY(),
                pose.getTranslation().getZ(),
                pose.getRotation().getQuaternion().getW(),
                pose.getRotation().getQuaternion().getX(),
                pose.getRotation().getQuaternion().getY(),
                pose.getRotation().getQuaternion().getZ()
            });
  }

  public Pose2d getBotPose() {
    Pose2d temp = new Pose2d();
    if (DriverStation.getAlliance() == Alliance.Blue)
      temp = LimelightHelpers.getBotPose2d_wpiBlue(m_llv.activeName);
    else
      temp = LimelightHelpers.getBotPose2d_wpiRed(m_llv.activeName);
    if (poseIsOnField(temp) && poseChangeInRange(temp))
      return temp;
    else
      return temp;
  }

  public boolean poseIsOnField(Pose2d pose) {
    return pose.getX() <= SimConstants.fieldLength &&
        pose.getY() <= SimConstants.fieldWidth
        && pose.getRotation().getDegrees() <= 180
        && pose.getRotation().getDegrees() >= -180;
  }

  public boolean poseChangeInRange(Pose2d pose) {
    return true;
    // return Math.abs(pose.getX() - lastPose.getX()) < maxXChange
    // && Math.abs(pose.getY() - lastPose.getY()) < maxYChange;
  }

  public Pose3d getRobotPose_FieldSpace() {
    return LimelightHelpers.getBotPose3d(m_llv.activeName);
  }

  public Double[] getBotPoseForAdvScope(Pose3d pose) {
    return new Double[] {
        pose.getTranslation().getX(),
        pose.getTranslation().getY(),
        pose.getTranslation().getZ(),
        pose.getRotation().getQuaternion().getW(),
        pose.getRotation().getQuaternion().getX(),
        pose.getRotation().getQuaternion().getY(),
        pose.getRotation().getQuaternion().getZ()
    };

  }

}
