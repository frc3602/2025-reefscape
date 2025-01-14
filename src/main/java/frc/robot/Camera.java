package frc.robot;

import static frc.robot.Constants.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

public class Camera {
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    private double lastEstimateTimestamp = 0.0;

    public Camera(String cameraName, Transform3d robotToCameraTransform) {
        photonCamera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCameraTransform);

        configCamera();
    }

    private PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

  public Optional<EstimatedRobotPose> getEstimatedRobotPose() {
        var visionEstimate = photonPoseEstimator.update(this.getLatestResult());
        double latestTimestamp = getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstimateTimestamp) > 1e-5;

        if (newResult) lastEstimateTimestamp = latestTimestamp;

        return (Optional<EstimatedRobotPose>) visionEstimate;
    }

    public double getTargetHeight() {
        double targetHeight;
        
        var result = getLatestResult();
    
        if (result.hasTargets()) {
          targetHeight = kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
        } else {
          targetHeight = 0.0;
        }
    
        return targetHeight;
    }
    
    public double getTargetDistance() {
        double targetDistance;

        var result = getLatestResult();

        if (result.hasTargets()) {
            targetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
              kCameraPitch.in(Radians), Units.degreesToRadians(result.getBestTarget().getPitch()));
        } else {
            targetDistance = 0.0;
        }

        return targetDistance;
    }

    private void configCamera() {
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
}
