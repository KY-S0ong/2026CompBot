// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.structures;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
  /* Creates a new VisionSystem. */
  // private PhotonCamera pho1 = new PhotonCamera("_mainPho1");

  /*private AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private Transform3d cameraPose =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private PhotonPoseEstimator photonPoseEstimator =
      new PhotonPoseEstimator(fieldLayout, cameraPose);

  public VisionSystem() {
    /*UsbCamera x = CameraServer.startAutomaticCapture();
    x.setResolution(320, 240);
    x.setFPS(30);*/

  // }
  /*
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Optional<EstimatedRobotPose> addPhotonMeasurement() {
    var result = pho1.getLatestResult();
    var visionEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = photonPoseEstimator.estimateLowestAmbiguityPose(result);
      return visionEst;
    }
    return null;

  }*/
}
