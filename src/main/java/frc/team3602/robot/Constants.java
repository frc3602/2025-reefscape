/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
    public final static int kControlPanelPort = 1;
    public final static int kLeftLASERCANId = 55;
    public final static int kRightLASERCANId = 54;
  }

  public final class DrivetrainConstants {
    public final static int kAlignmentLASERCANId = 6;
    public final static double minMetersFromReef = 0.0;
    public final static double maxMetersFromReef = 1.0; // TODO: Use real numbers :(
    public final static RegionOfInterest leftmostRegion = new RegionOfInterest(2,8,4,4);
    public final static RegionOfInterest rightmostRegion = new RegionOfInterest(12,8,4,4);
  }

  // in case we want the flypath stuff in superstructure or something
  public final class flyPathPosesConstants {

    public static final Pose2d startingPose = new Pose2d(7.6, 0.5, Rotation2d.fromDegrees(180));

    // TODO- change with real life robot testing
    public static final Pose2d blueNONBargeCoralIntakePose = new Pose2d(1.22, 1.68, Rotation2d.fromDegrees(145));
    public static final Pose2d blueBargeCoralIntakePose = new Pose2d(0.8, 7.07, Rotation2d.fromDegrees(34.6));
    public static final Pose2d redNONBargeCoralIntakePose = new Pose2d(16.5, 7.5, Rotation2d.fromDegrees(-36.7));
    public static final Pose2d redBargeCoralIntakePose = new Pose2d(16.5, 1.68, Rotation2d.fromDegrees(-145));
  }

  public final class ElevatorConstants {
    public final static int kElevatorMotorId = 53;
    public final static int kElevatorFollowerId = 57;

    public final static double tolerance = 3;

    public final static double pivotStowHeight = 13.0;
    // Pre set heights

    public final static double coralIntakeHeight = 0.1;
    public final static double scoreLevelOne = 2;//2
    public final static double scoreLevelTwo = 6.5;//6
    public final static double scoreLevelThree = 16;//16
    public final static double scoreLevelFour = 33;//31.7, 31.5, 33 also 33 is max height!!!!!
    public final static double down = 0.1;

    // the rest of these are bs numbers still
    public final static double scoreAlgaeProcesser = 2;//2
    public final static double scoreAlgaeBarge = 4;//4
    public final static double removeAlgaeHigh = 13;//10
    public final static double removeAlgaeLow = 22;//20

    // PID Constants
    public final static double KP = 0.6;
    public final static double KI = 0.0;
    public final static double KD = 0.05;

    // ffe Constants
    public final static double KS = 5.0;
    public final static double KG = 0.25;
    public final static double KV = 0.9;
    public final static double KA = 0.1;

    // simulation constants
    public final static double kMaxHeightMeters = 1.5;
    // sim elevator PID constants
    public final static double simKP = 2;
    public final static double simKI = 0;
    public final static double simKD = 0.01;

    // sim elevator ffe constants
    public final static double simKS = 4.0;
    public final static double simKG = 0;
    public final static double simKV = 0.4;
    public final static double simKA = 0.1;

  }

  public final class IntakeConstants {
    public final static int kIntakeMotorId = 56;
    public final static double coralSpeed = .50;
    public final static double intakeAlgaeSpeed = -0.5;// -0.3
    public final static double scorAlgeaSpeed = 0.3;
  }

  public final class PivotConstants {
    public final static int kPivotMotorId = 52;

    public final static int kPivotEncoderId = 34;

    public final static double tolerance = 5; // TODO - this is random still

    public final static double coralIntakeAngle = 101;// 100

    public final static double stowAngle = 85;
    public final static double l4StowAngle = 20;
    public final static double holdAlgaeAngle = 29;
    public final static double scoreL4Angle = 80;//85, 83
    public final static double scoreCoralAngle = 80;
    public final static double intakeAlgaeAngle = -48;
    public final static double scoreAlgaeProcesserAngle = -42;
    public final static double bargeAngle = 00;

    // PID Constants
    public final static double KP = 0.09;// .056 //.09 .11
    public final static double KI = 0.0;
    public final static double KD = 0.0001;

    // ffe Constants
    public final static double KS = 0.8;
    public final static double KG = 0.27;// .25
    public final static double KV = 0.9;
    public final static double KA = 0.1;

    // Simulation Constants
    public final static int gearing = 36;
    public final static double lengthMeters = 0.5;
    public final static double massKg = 3.0;

    // sim pivot PID constants
    public final static double simPivotKP = 0.25;
    public final static double simPivotKI = 0;
    public final static double simPivotKD = 0.01;

    // sim pivot ffe constants
    public final static double simPivotKS = 4.0;
    public final static double simPivotKG = 1.315; // 1.4> -> 1.3<
    public final static double simPivotKV = 0.4;
    public final static double simPivotKA = 0.1;
  }

  public final class ClimberConstants {
    public static final int motorCANId = 18;
    public static final double percentVoltageScalar = 0.4; // 0.2
  }

  public final class VisionConstants {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final int kWidthOfCamera = 4656;
    public static final int kHeightOfCamera = 3496;
    public static final Rotation2d kCameraFOV = Rotation2d.fromDegrees(90.0);

    public static final String kMod0CameraName = "mod0Camera";
    public static final String kMod1CameraName = "mod1Camera";
    public static final String kMod2CameraName = "mod2Camera";
    public static final String kMod3CameraName = "mod3Camera";

    // TODO
    // TRANFORM 3D VALUES
    // ARE MADE UP
    // THEY MUST BE CHANGED eventually
    // I believe x&y coordinates are meters
    public static final Transform3d kRobotToMod0CameraTransform = new Transform3d(
        new Translation3d(-0.254, 0.254, 0.15),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(135.0)));
    public static final Transform3d kRobotToMod1CameraTransform = new Transform3d(
        new Translation3d(0.254, 0.254, 0.15),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(45.0)));
    public static final Transform3d kRobotToMod2CameraTransform = new Transform3d(
        new Translation3d(0.254, -0.254, 0.15),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(315.0)));
    public static final Transform3d kRobotToMod3CameraTransform = new Transform3d(
        new Translation3d(-0.254, -0.254, 0.15),
        new Rotation3d(0.0, 0.0, Units.degreesToRadians(225.0)));

    public static final Measure<DistanceUnit> kCameraHeight = Inches.of(4);
    public static final Measure<AngleUnit> kCameraPitch = Degrees.of(45); // 23.5

    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

}