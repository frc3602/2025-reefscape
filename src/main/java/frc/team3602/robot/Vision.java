/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static frc.team3602.robot.Constants.VisionConstants.*;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;

public class Vision {
    /* Camerae */
    public final PhotonCamera mod0Camera = new PhotonCamera("mod0Cam");
    public final PhotonCamera mod1Camera = new PhotonCamera("mod1Cam");
    public final PhotonCamera mod2Camera = new PhotonCamera("mod2Cam");
    public final PhotonCamera mod3Camera = new PhotonCamera("mod3Cam");

    /* Field Simulation */
    private final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
    // public final VisionSystemSim visionSim = new VisionSystemSim("ALL_CAMS");

    /* Pose Estimators */
    private final PhotonPoseEstimator photonPoseEstimator0 = new PhotonPoseEstimator(kFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY, kRobotToMod0CameraTransform);
    private final PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(kFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY, kRobotToMod1CameraTransform);
    private final PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(kFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY, kRobotToMod2CameraTransform);
    private final PhotonPoseEstimator photonPoseEstimator3 = new PhotonPoseEstimator(kFieldLayout,
            PoseStrategy.LOWEST_AMBIGUITY, kRobotToMod3CameraTransform);

    /* Camera Simulation */
    public final VisionSystemSim visionSim = new VisionSystemSim("Vision Sim");
    private final SimCameraProperties cameraProperties = new SimCameraProperties();
    private PhotonCameraSim camera0Sim = new PhotonCameraSim(mod0Camera,
            cameraProperties);
    private PhotonCameraSim camera1Sim = new PhotonCameraSim(mod1Camera,
            cameraProperties);
    private PhotonCameraSim camera2Sim = new PhotonCameraSim(mod2Camera,
            cameraProperties);
    private PhotonCameraSim camera3Sim = new PhotonCameraSim(mod3Camera,
            cameraProperties);

    /* Constructor */
    public Vision() {
        // Configure the pose estimators
        photonPoseEstimator0.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Generate a VisionSystemSim
        visionSim.addAprilTags(kFieldLayout);
        visionSim.addCamera(camera0Sim, kRobotToMod0CameraTransform);
        visionSim.addCamera(camera1Sim, kRobotToMod1CameraTransform);
        visionSim.addCamera(camera2Sim, kRobotToMod2CameraTransform);
        visionSim.addCamera(camera3Sim, kRobotToMod3CameraTransform);

        // TODO: Actually, calibrate camerae
        // Calibrate camerae
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(20);
        cameraProperties.setAvgLatencyMs(35.0);
        cameraProperties.setLatencyStdDevMs(5);
        cameraProperties.setCalibration(kWidthOfCamera, kHeightOfCamera, kCameraFOV);
    }

    /* Calculations for pose estimations */
    public double lastMod0EstimateTimestamp = 0.0;

    public Optional<EstimatedRobotPose> getMod0EstimatedPose() {
        Optional<EstimatedRobotPose> Mod0RobotPose = Optional.empty();

        for (PhotonPipelineResult change : mod0Camera.getAllUnreadResults()) {

            Mod0RobotPose = photonPoseEstimator0.update(change);
        }

        lastMod0EstimateTimestamp = mod0Camera.getLatestResult().getTimestampSeconds();

        return Mod0RobotPose;
    }

    /* Calculations for pose estimations */
    public double lastMod1EstimateTimestamp = 0.0;

    public Optional<EstimatedRobotPose> getMod1EstimatedPose() {
        Optional<EstimatedRobotPose> Mod1RobotPose = Optional.empty();

        for (PhotonPipelineResult change : mod1Camera.getAllUnreadResults()) {

            Mod1RobotPose = photonPoseEstimator1.update(change);
        }

        lastMod1EstimateTimestamp = mod1Camera.getLatestResult().getTimestampSeconds();

        return Mod1RobotPose;
    }

    /* Calculations for pose estimations */
    public double lastMod2EstimateTimestamp = 0.0;

    public Optional<EstimatedRobotPose> getMod2EstimatedPose() {
        Optional<EstimatedRobotPose> Mod2RobotPose = Optional.empty();

        for (PhotonPipelineResult change : mod2Camera.getAllUnreadResults()) {

            Mod2RobotPose = photonPoseEstimator2.update(change);
        }

        lastMod2EstimateTimestamp = mod2Camera.getLatestResult().getTimestampSeconds();

        return Mod2RobotPose;
    }

    /* Calculations for pose estimations */
    public double lastMod3EstimateTimestamp = 0.0;

    public Optional<EstimatedRobotPose> getMod3EstimatedPose() {
        Optional<EstimatedRobotPose> Mod3RobotPose = Optional.empty();

        for (PhotonPipelineResult change : mod3Camera.getAllUnreadResults()) {

            Mod3RobotPose = photonPoseEstimator3.update(change);
        }

        lastMod3EstimateTimestamp = mod3Camera.getLatestResult().getTimestampSeconds();

        return Mod3RobotPose;
    }

    public void updateViz(Pose2d pose) {
        visionSim.update(pose);
    }

    public void reset() {
        visionSim.clearAprilTags();
        visionSim.addAprilTags(kFieldLayout);
    }

}