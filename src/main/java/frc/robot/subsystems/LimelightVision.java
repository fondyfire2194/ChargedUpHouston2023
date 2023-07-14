// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  public Pose2d visionPoseEstimatedData;

  public double imageCaptureTime;

  public int fiducialId;

  public double[] llHeartbeat = { 0, 0 };

  public double[] llHeartbeatLast = { 0, 0 };

  public boolean allianceBlue;

  public enum pipelinetype {
    retroreflective,
    grip,
    python,
    fiducialmarkers,
    classifier,
    detector;

    public static final pipelinetype values[] = values();
  }

  // camera ppeline names are 15
  public enum pipelines {
    REFL_LOW_TAPE(0, pipelinetype.retroreflective),
    REFL_HIGH_TAPE(1, pipelinetype.retroreflective),
    RED_NO_BUMP(2, pipelinetype.fiducialmarkers),
    BLUE_NO_BUMP(3, pipelinetype.fiducialmarkers),
    RED_BUMP(4, pipelinetype.fiducialmarkers),
    BLUE_BUMP(5, pipelinetype.fiducialmarkers),
    FID_MARKERS(6, pipelinetype.fiducialmarkers),
    SPARE7(7, pipelinetype.fiducialmarkers),
    CUBE_DETECT(8, pipelinetype.detector),
    CONE_DETECT(9, pipelinetype.detector);

    public static final pipelines values[] = values();

    private int number;

    private pipelinetype type;

    public String pipelineTypeName;

    private pipelines(int number, pipelinetype type) {
      this.number = number;
      this.type = type;
    }

  }

  public int currentPipelineIndex;
  public pipelinetype currentPipelineType;
  public pipelines currentPipeline;

  private int[] samples = { 0, 0 };

  public boolean[] limelightExists = { false, false };

  public String limelighttypename = "fiducial";

  private String[] m_name = new String[2];

  private int numCameras = 2;

  public int activeLimelight;

  public String activeName;

  public LimelightVision(String[] name) {
    currentPipeline = pipelines.FID_MARKERS;
    m_name[0] = name[0];
    m_name[1] = name[1];

    activeLimelight = 0;

  }

  public boolean getAllianceBlue() {
    return (DriverStation.getAlliance() == Alliance.Blue);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ActLL", activeLimelight);
    activeName = m_name[activeLimelight];
    SmartDashboard.putString("LLAN", activeName);
    if (RobotBase.isReal()) {
      for (int i = 0; i < numCameras; i++) {
        llHeartbeat[i] = LimelightHelpers.getLimelightNTDouble(m_name[i], "hb");
        if (llHeartbeat[i] == llHeartbeatLast[i]) {
          samples[i] += 1;
        } else {
          samples[i] = 0;
          llHeartbeatLast[i] = llHeartbeat[i];
          limelightExists[i] = true;
        }
        if (samples[i] > 5)
          limelightExists[i] = false;
      }
    }
    // SmartDashboard.putBoolean("LLExists", limelightExists);

    if (RobotBase.isReal()) {

      fiducialId = (int) LimelightHelpers.getFiducialID(m_name[activeLimelight]);
      SmartDashboard.putNumber("FIDID", fiducialId);
      currentPipelineIndex = (int) LimelightHelpers.getCurrentPipelineIndex(m_name[activeLimelight]);

      currentPipeline = pipelines.values[currentPipelineIndex];

      currentPipelineType = currentPipeline.type;

      limelighttypename = getCurrentPipelineTypeName();
    }
  }

  public void setActiveCamera(int activeCam) {
    activeLimelight = activeCam;
    activeName = m_name[activeLimelight];
  }

  public double round2dp(double number) {
    number = Math.round(number * 100);
    number /= 100;
    return number;
  }

  public String getCurrentPipelineName() {
    return currentPipeline.name();
  }

  public String getCurrentPipelineTypeName() {
    return currentPipeline.type.name();
  }

  public pipelinetype getCurrentPipelineType() {
    return currentPipeline.type;
  }

  public pipelines getCurrentPipeline() {
    return currentPipeline;
  }

  public void setRedNoBumpPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.RED_NO_BUMP.ordinal());
  }

  public void setBlueNoBumpPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.BLUE_NO_BUMP.ordinal());
  }

  public void setRedBumpPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.RED_BUMP.ordinal());
  }

  public void setBlueBumpPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.BLUE_BUMP.ordinal());
  }

  public void setHighTapePipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.REFL_HIGH_TAPE.ordinal());
  }

  public void setLowTapePipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.REFL_LOW_TAPE.ordinal());
  }

  public void setConeDetectorPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.CONE_DETECT.ordinal());
  }

  public void setCubeDetectorPipeline() {
    if (limelightExists[activeLimelight])
      LimelightHelpers.setPipelineIndex(activeName, pipelines.CUBE_DETECT.ordinal());
  }

}
