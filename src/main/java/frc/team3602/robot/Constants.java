/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
    public final static int kControlPanelPort = 1;
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
    public final static double KP = 0.076;//.056 //.0722
    public final static double KI = 0.0;
    public final static double KD = 0;

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

}