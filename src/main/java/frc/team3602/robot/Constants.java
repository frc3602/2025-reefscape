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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
    public final static int kControlPanelPort = 1;
  }

  public final class DrivetrainConstants {
    public final static int kAlignmentLASERCANId = 6;
  }

  // in case we want the flypath stuff in superstructure or something
  public final class flyPathPosesConstants {
    
    public static double robotWidth = Units.inchesToMeters(28 + 6);
    public static double robotLength = Units.inchesToMeters(30 + 6);
    public static Transform2d halfRobot = new Transform2d(robotLength / 2.0, 0, new Rotation2d());

    public static Transform2d halfRobotGatherLeftFar =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(16),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherLeftClose =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(-16),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherRightFar =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(-16),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherRightClose =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(16),
                    Rotation2d.kZero);

    public static Transform2d halfRobotCoralRight =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(3.25),
                    Units.inchesToMeters(5.5),
                    Rotation2d.kZero);
    public static Transform2d halfRobotCoralLeft =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(3.5),
                    Units.inchesToMeters(-8),
                    Rotation2d.kZero);

    public static Transform2d halfRobotAlgae =
            new Transform2d(robotLength / 2.0 + Units.inchesToMeters(6), 0, Rotation2d.kZero);

    // TODO: make code that uses this
    public static Transform2d halfRobotCoralLevel1 =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(7.5),
                    Units.inchesToMeters(0),
                    new Rotation2d());

    public static AprilTagFieldLayout tags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    static Pose2d[] blueStarts = {
        new Pose2d(6, 3, Rotation2d.k180deg),
        new Pose2d(6, 3, Rotation2d.k180deg),
        new Pose2d(6, 3, Rotation2d.k180deg)
    };

    static Pose2d[] redStarts = {
        new Pose2d(12, 3, Rotation2d.kZero),
        new Pose2d(12, 3, Rotation2d.kZero),
        new Pose2d(12, 3, Rotation2d.kZero)
    };
    
    public static final Pose2d APose = new Pose2d(if (isBlue()) {
      return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralLeft));
  } else {
      return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft));
  });

    public static final Pose2d BPose = new Pose2d(if (isBlue()) {
      return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralRight));
  } else {
      return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralRight));
  });

  public static final Pose2d CPose = new Pose2d(if (isBlue()) {
    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralLeft));
} else {
    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralLeft));
});

public static final Pose2d DPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralRight));
} else {
  return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralRight));
});

public static final Pose2d EPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralLeft));
} else {
  return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralLeft));
});

public static final Pose2d FPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralRight));
} else {
  return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralRight));
});

public static final Pose2d CPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralLeft));
} else {
  return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralLeft));
});

public static final Pose2d DPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralRight));
} else {
  return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralRight));
});

public static final Pose2d CPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralLeft));
} else {
  return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralLeft));
});

public static final Pose2d DPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralRight));
} else {
  return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralRight));
});

public static final Pose2d CPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralLeft));
} else {
  return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralLeft));
});

public static final Pose2d DPose = new Pose2d(if (isBlue()) {
  return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralRight));
} else {
  return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralRight));
});





 

    public static Pose2d getStartLoc(int idx) {
        if (isBlue()) {
            if (idx >= blueStarts.length) return null;
            return blueStarts[idx];
        } else {
            if (idx >= redStarts.length) return null;
            return redStarts[idx];
        }
    }

    public static Translation2d getReef() {
      Pose2d front, back;
      if (isBlue()) {
          front = tags.getTagPose(18).get().toPose2d();
          back = tags.getTagPose(21).get().toPose2d();
      } else {
          front = tags.getTagPose(7).get().toPose2d();
          back = tags.getTagPose(10).get().toPose2d();
      }

      return front.getTranslation().plus(back.getTranslation()).times(0.5);
  }



public static Pose2d getLeftGatherStationFar() {
  if (!isBlue()) {
      return invert(tags.getTagPose(1).get().toPose2d().plus(halfRobotGatherLeftFar));
  } else {
      return invert(tags.getTagPose(13).get().toPose2d().plus(halfRobotGatherLeftFar));
  }
}

public static Pose2d getRightGatherStationFar() {
  if (!isBlue()) {
      return invert(tags.getTagPose(2).get().toPose2d().plus(halfRobotGatherRightFar));
  } else {
      return invert(tags.getTagPose(12).get().toPose2d().plus(halfRobotGatherRightFar));
  }
}

public static Pose2d getLeftGatherStationClose() {
  if (!isBlue()) {
      return invert(tags.getTagPose(1).get().toPose2d().plus(halfRobotGatherLeftClose));
  } else {
      return invert(tags.getTagPose(13).get().toPose2d().plus(halfRobotGatherLeftClose));
  }
}

public static Pose2d getRightGatherStationClose() {
  if (!isBlue()) {
      return invert(tags.getTagPose(2).get().toPose2d().plus(halfRobotGatherRightClose));
  } else {
      return invert(tags.getTagPose(12).get().toPose2d().plus(halfRobotGatherRightClose));
  }
}



public static Pose2d getTag7() {
  Pose2d tag = tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft);
  return invert(tag);
}

public static boolean isBlue() {
  return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
}

public static Pose2d invert(Pose2d in) {
  return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg));
}

public static Pose2d getProcLoc() {
  if (isBlue()) {
      return invert(tags.getTagPose(16).get().toPose2d().plus(halfRobot));
  } else {
      return invert(tags.getTagPose(3).get().toPose2d().plus(halfRobot));
  }
}



    // // TODO- change with real life robot testing
    // public static final Pose2d blueNONBargeCoralIntakePose = new Pose2d(1.22, 1.68, Rotation2d.fromDegrees(145));
    // public static final Pose2d blueBargeCoralIntakePose = new Pose2d(0.8, 7.07, Rotation2d.fromDegrees(34.6));
    // public static final Pose2d redNONBargeCoralIntakePose = new Pose2d(16.5, 7.5, Rotation2d.fromDegrees(-36.7));
    // public static final Pose2d redBargeCoralIntakePose = new Pose2d(16.5, 1.68, Rotation2d.fromDegrees(-145));
  }

  public final class ElevatorConstants {
    public final static int kElevatorMotorId = 53;
    public final static int kElevatorFollowerId = 57;

    public final static double tolerance = 3;

    public final static double pivotStowHeight = 13.0;
    // Pre set heights

    public final static double coralIntakeHeight = 0.1;
    public final static double scoreLevelOne = 2;
    public final static double scoreLevelTwo = 6;
    public final static double scoreLevelThree = 16;
    public final static double scoreLevelFour = 30.7;
    public final static double down = 0.1;

    //the rest of these are bs numbers still
    public final static double scoreAlgaeProcesser = 2;
    public final static double scoreAlgaeBarge = 4;
    public final static double removeAlgaeHigh = 10;
    public final static double removeAlgaeLow = 20;

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
    public final static double coralSpeed = 1.0;
    public final static double intakeAlgaeSpeed = -0.5;//-0.3
    public final static double scorAlgeaSpeed = 0.3;
  }

  public final class PivotConstants {
    public final static int kPivotMotorId = 52;

    public final static int kPivotEncoderId = 34;

    public final static double tolerance = 5; // TODO - this is random still

    public final static double coralIntakeAngle = 101;//100
    
    public final static double lowStowAngle = 86;//87
    public final static double highStowAngle = 30;//35
    public final static double scoreL4Angle = 89;
    public final static double scoreCoralAngle = 80;
    public final static double intakeAlgaeAngle = -50;
    public final static double scoreAlgaeProcesserAngle = -90;


    // PID Constants
    public final static double KP = 0.11;//.056 //.09 .11
    public final static double KI = 0.0;
    public final static double KD = 0.0001;

    // ffe Constants
    public final static double KS = 0.8;
    public final static double KG = 0.27;//.25
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

  public final class VisionConstants {
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

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

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

}