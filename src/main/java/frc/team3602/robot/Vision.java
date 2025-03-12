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
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.team3602.robot.Constants.VisionConstants.*;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;

public class Vision {
    private DrivetrainSubsystem driveSubsys;

/*Cameras */
  public final PhotonCamera mod0Camera = new PhotonCamera("mod0Cam");
  public final PhotonCamera mod1Camera = new PhotonCamera("mod1Cam");
  public final PhotonCamera mod2Camera = new PhotonCamera("mod2Cam");
  public final PhotonCamera mod3Camera = new PhotonCamera("mod3Cam");

    /* Field Simulation */
    private final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
    // public final VisionSystemSim visionSim = new VisionSystemSim("ALL_CAMS");

    /* Pose Estimators */
    // private final PhotonPoseEstimator photonPoseEstimator0 = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToMod0CameraTransform);
    // private final PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToMod1CameraTransform);
    private final PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToMod2CameraTransform);
    // private final PhotonPoseEstimator photonPoseEstimator3 = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToMod3CameraTransform);

    // /* Camera Simulation */
    // private final SimCameraProperties cameraProperties = new SimCameraProperties();
    //   private PhotonCameraSim camera0Sim = new PhotonCameraSim(mod0Camera, cameraProperties);
    //   private PhotonCameraSim camera1Sim = new PhotonCameraSim(mod1Camera, cameraProperties);
    //   private PhotonCameraSim camera2Sim = new PhotonCameraSim(mod2Camera, cameraProperties);
    //   private PhotonCameraSim camera3Sim = new PhotonCameraSim(mod3Camera, cameraProperties);


    /* Vision Constructor */      
    public Vision(DrivetrainSubsystem driveSubsys) {
        this.driveSubsys = driveSubsys;
        // photonPoseEstimator0.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        // photonPoseEstimator3.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // visionSim.addAprilTags(kFieldLayout);
        // visionSim.addCamera(camera0Sim, kRobotToMod0CameraTransform);
        // visionSim.addCamera(camera1Sim, kRobotToMod1CameraTransform);
        // visionSim.addCamera(camera2Sim, kRobotToMod2CameraTransform);
        // visionSim.addCamera(camera3Sim, kRobotToMod3CameraTransform);

        // cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // cameraProperties.setCalibError(0.25, 0.08);
        // cameraProperties.setFPS(20);
        // cameraProperties.setAvgLatencyMs(35.0);
        // cameraProperties.setLatencyStdDevMs(5);
        // cameraProperties.setCalibration(kWidthOfCamera, kHeightOfCamera, kCameraFOV);
    }

    public void reset() {
        // visionSim.clearAprilTags();
        // visionSim.addAprilTags(kFieldLayout);
    }

    // private double lastEstimateTimestampMod0 = 0.0;
    // private Pose2d mod0Pose2d;// = new Pose2d();

    // public Optional<EstimatedRobotPose> getEstimatedPoseMod0(Pose2d prevEstimatedRobotPose){
    //     double latestTimestamp = mod0Camera.getLatestResult().getTimestampSeconds();
    //     boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestampMod0) > 1e-5;

    //     if (newResult){
    //         lastEstimateTimestampMod0 = latestTimestamp;
    //     }else{
    //         Commands.print("mod 0 camera is old, moldy, and stale!!!");
    //     }


    //     photonPoseEstimator0.setReferencePose(prevEstimatedRobotPose);
    //     mod0Pose2d = photonPoseEstimator0.update(mod0Camera.getLatestResult()).get().estimatedPose.toPose2d();
    //     return photonPoseEstimator0.update(mod0Camera.getLatestResult());

    // }

    // private double lastEstimateTimestampMod1 = 0.0;
    // private Pose2d mod1Pose2d;// = new Pose2d();

    // public Optional<EstimatedRobotPose> getEstimatedPoseMod1(Pose2d prevEstimatedRobotPose){
    //     double latestTimestamp = mod1Camera.getLatestResult().getTimestampSeconds();
    //     boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestampMod1) > 1e-5;

    //     if (newResult){
    //         lastEstimateTimestampMod1 = latestTimestamp;
    //     }else{
    //         Commands.print("mod 1 camera is old, moldy, and stale!!!");
    //     }


    //     photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
    //     mod1Pose2d = photonPoseEstimator1.update(mod1Camera.getLatestResult()).get().estimatedPose.toPose2d();
    //     return photonPoseEstimator1.update(mod1Camera.getLatestResult());

    // }
    private double lastEstimateTimestampMod2 = 0.0;
    private Pose2d mod2Pose2d;// = new Pose2d();

    public Optional<EstimatedRobotPose> getEstimatedPoseMod2(Pose2d prevEstimatedRobotPose){
        double latestTimestamp = mod2Camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestampMod2) > 1e-5;

        if (newResult){
            lastEstimateTimestampMod2 = latestTimestamp;
        }else{
            Commands.print("mod 2 camera is old, moldy, and stale!!!");
        }

        photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
        mod2Pose2d = photonPoseEstimator2.update(mod2Camera.getLatestResult()).get().estimatedPose.toPose2d();
        return photonPoseEstimator2.update(mod2Camera.getLatestResult());

    }

//     private double lastEstimateTimestampMod3 = 0.0;
//     private Pose2d mod3Pose2d;// = new Pose2d();

//     public Optional<EstimatedRobotPose> getEstimatedPoseMod3(Pose2d prevEstimatedRobotPose){
//         double latestTimestamp = mod3Camera.getLatestResult().getTimestampSeconds();
//         boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestampMod3) > 1e-5;

//         if (newResult){
//             lastEstimateTimestampMod3 = latestTimestamp;
//         }else{
//             Commands.print("mod 3 camera is old, moldy, and stale!!!");
//         }


//         photonPoseEstimator3.setReferencePose(prevEstimatedRobotPose);
//         mod3Pose2d = photonPoseEstimator3.update(mod3Camera.getLatestResult()).get().estimatedPose.toPose2d();
//         return photonPoseEstimator3.update(mod3Camera.getLatestResult());

//     }

//   public void update(Pose2d pose) {
//     try {
//         getEstimatedPoseMod0(pose);
//         driveSubsys.addVisionMeasurement(mod0Pose2d, lastEstimateTimestampMod0);
//     } catch (Exception e) {
//        // mod0Pose2d = pose;
//         Commands.print("mod0 pose failed");
//     }

//     try {
//         getEstimatedPoseMod1(pose);
//         driveSubsys.addVisionMeasurement(mod1Pose2d, lastEstimateTimestampMod1);
//     } catch (Exception e) {
//         //mod1Pose2d = pose;
//         Commands.print("mod1 pose failed");
//     } 
    
//     try {
//         getEstimatedPoseMod2(pose);
//         driveSubsys.addVisionMeasurement(mod2Pose2d, lastEstimateTimestampMod2);
//     } catch (Exception e) {
//        // mod2Pose2d = pose;
//         Commands.print("mod2 pose failed");
//     } 
   
//     try {
//          getEstimatedPoseMod3(pose);
//          driveSubsys.addVisionMeasurement(mod3Pose2d, lastEstimateTimestampMod3);
//     } catch (Exception e) {
//          //mod3Pose2d = pose;
//          Commands.print("mod3 pose failed");
//     }    
   

    // SmartDashboard.putNumber("posex", pose.getX());
    // SmartDashboard.putNumber("posey", pose.getY());

    //visionSim.update(pose);
    //isionSim.getDebugField();
  //}
}