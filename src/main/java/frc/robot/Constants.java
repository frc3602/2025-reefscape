/*
 * Copyright (C) 2023 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
    public final class OperatorInterfaceConstants {
      public final static int kXboxControllerPort = 0;
      public final static int kControlPanelPort = 1;
    }

    public final class VisionConstants {
      public static final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

      public static final String kMod0CameraName = "mod0Camera";
      public static final String kMod1CameraName = "mod1Camera";
      public static final String kMod2CameraName = "mod2Camera";
      public static final String kMod3CameraName = "mod3Camera";

      public static final Transform3d kRobotToMod0Camera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
      );
      public static final Transform3d kRobotToMod1Camera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
      );
      public static final Transform3d kRobotToMod2Camera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
      );
      public static final Transform3d kRobotToMod3Camera = new Transform3d(
        new Translation3d(0.0, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, 0.0)
      );

      public static final Measure<DistanceUnit> kCameraHeight = Inches.of(4);
      public static final Measure<AngleUnit> kCameraPitch = Degrees.of(45); //23.5

      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public final class GripperConstants {
        //pivot PID constants
        public final static double pivotKP = 1;
        public final static double pivotKI = 0;
        public final static double pivotKD = 0;

        //pivot ffe constants
        public final static double pivotKS = 5;
        public final static double pivotKG = 2;
        public final static double pivotKV = 0.9;
        public final static double pivotKA = 0.1;

         // sim pivot PID constants
         public final static double simPivotKP = 9;
         public final static double simPivotKI = 0;
         public final static double simPivotKD = 0;
 
         // sim pivot ffe constants
         public final static double simPivotKS = 5;
         public final static double simPivotKG = 2;
         public final static double simPivotKV = 0.9;
         public final static double simPivotKA = 0.1;

         //sim constants
         public final static int pivotGearing = 36;
         public final static double pivotLengthMeters = 0.5;
         public final static double pivotMassKg = 3;
    }
}
