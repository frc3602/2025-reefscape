/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;

import static frc.team3602.robot.Constants.VisionConstants.*;

public class VisionSimulation {
    
    /* Field Simulation */
    private final VisionSystemSim visionSimulation = new VisionSystemSim("main");

    /* Camera Simulation */
    private final SimCameraProperties cameraProperties = new SimCameraProperties();
    private List<Camera> camerae = new ArrayList<Camera>(4);

    public VisionSimulation() {
        visionSimulation.addAprilTags(kFieldLayout);

        cameraProperties.setCalibration(kWidthOfCamera, kHeightOfCamera, kCameraFOV);
    }

    public VisionSimulation addCamera(Camera camera) {
        int numberOfCamerae = camerae.size();
        camerae.add(camera);
        visionSimulation.addCamera(camerae.get(numberOfCamerae).getSimulation(cameraProperties), camerae.get(numberOfCamerae).getRobotToCameraTransform());

        return this;
    }

    public void reset() {
        visionSimulation.clearAprilTags();
        visionSimulation.addAprilTags(kFieldLayout);
    }

    public void update(Pose2d pose) {
        visionSimulation.update(pose);
    }
}