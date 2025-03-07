/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import au.grapplerobotics.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.Constants.ElevatorConstants;
import frc.team3602.robot.Constants.VisionConstants;
import frc.team3602.robot.Constants.flyPathPosesConstants;
import frc.team3602.robot.generated.TunerConstants;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;

import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.kFieldLayout;

import java.util.Collections;
import java.util.List;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* laser can */
  private LaserCan lc;

  /* Operator interfaces */
  private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  // for simulation
  //private final CommandJoystick joystick = new CommandJoystick(0);
  private final CommandJoystick joystick = new CommandJoystick(1);

  /* Subsystems */
  private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem(
      elevatorSubsys.elevatorSimMech.getRoot("Pivot Root", 0.75, 0.7), () -> elevatorSubsys.elevatorViz.getLength());
  private final IntakeSubsystem intakeSubsys = new IntakeSubsystem(
      elevatorSubsys.elevatorSimMech.getRoot("Intake Wheel Root", 0.75, 0.3),
      () -> elevatorSubsys.elevatorViz.getLength(), () -> pivotSubsys.pivotSim.getAngleRads());

  private final Vision vision = new Vision(drivetrainSubsys);
  private final Superstructure superstructure = new
  Superstructure(drivetrainSubsys, elevatorSubsys, intakeSubsys, pivotSubsys, vision
  );

  /* Autonomous */
  private final SendableChooser<Command> autoChooser;// = new SendableChooser<>();
  private SendableChooser<Double> polarityChooser = new SendableChooser<>();

  public RobotContainer() {

     //auton commands
    NamedCommands.registerCommand("elevDown", elevatorSubsys.setHeight(ElevatorConstants.down));
    NamedCommands.registerCommand("prepElevL4", superstructure.autonPrepElevL4());
    NamedCommands.registerCommand("prepElevL3", superstructure.autonPrepElevL3());
    NamedCommands.registerCommand("prepElevL2", superstructure.autonPrepElevL2());
    NamedCommands.registerCommand("prepElevL1", superstructure.autonPrepElevL1());
    NamedCommands.registerCommand("prepElevCoralIntake", superstructure.autonPrepElevCoralIntake());
    NamedCommands.registerCommand("down", superstructure.down());
    NamedCommands.registerCommand("prepPivotL4", superstructure.autonPrepPivotL4());
    NamedCommands.registerCommand("prepPivotReef", superstructure.autonPrepPivotReef());
    NamedCommands.registerCommand("prepPivotCoralIntake", superstructure.autonPrepPivotCoralIntake());
    NamedCommands.registerCommand("prepPivotAlgae", superstructure.autonPrepPivotAlgae());

    NamedCommands.registerCommand("shoot", superstructure.autonShoot());
    NamedCommands.registerCommand("intake", superstructure.autonIntake());
    
    NamedCommands.registerCommand("grabAlgaeHigh", superstructure.autonGrabAlgaeHigh());
    NamedCommands.registerCommand("grabAlgaeLow", superstructure.autonGrabAlgaeLow());
    NamedCommands.registerCommand("holdAlgae", superstructure.autonHoldAlgae());


    drivetrainSubsys.configDrivetrainSubsys();
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Drive Polarity", polarityChooser);
    polarityChooser.setDefaultOption("Default", 1.0);
    polarityChooser.addOption("Positive", 1.0);
    polarityChooser.addOption("Negative", -1.0);


    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  public double driveLimiter;

  private void configDefaultCommands() {

    if (Utils.isSimulation()) {
      // drivetrainSubsys.setDefaultCommand(
      //     drivetrainSubsys.applyRequest(() -> drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed) // Drive forward
      //                                                                                                 // with negative Y
      //                                                                                                 // (forward)
      //         .withVelocityY(joystick.getRawAxis(1) * MaxSpeed) // Drive left with negative X (left)
      //         .withRotationalRate(-joystick2.getRawAxis(1) * MaxAngularRate)) // Drive counterclockwise with negative X
      //                                                                         // (left)
      // );
    } else {
      drivetrainSubsys.setDefaultCommand(
          drivetrainSubsys.applyRequest(() -> drive.withVelocityX( 0.5 * polarityChooser.getSelected() * -xboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
              .withVelocityY(0.5 * polarityChooser.getSelected() * -xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative
                                                                                // X (left)
          ));
    }
  }

  /**
   * Function that is called in the constructor where we configure operator
   * interface button bindings.
   */
  private void configButtonBindings() {
    if (Utils.isSimulation()) {

      // // joystick.button(1).whileTrue(elevatorSubsys.setHeight(0.0));
      // // joystick.button(2).onTrue(elevatorSubsys.setHeight(1.0));
      // // // joystick.button(3).onTrue(superstructure.scoreCoral());
      // // joystick.button(4).onTrue(drivetrainSubsys.flypathToCoralStation());

      // joystick2.button(1).onTrue(pivotSubsys.setAngle(-90));
      // joystick2.button(2).onTrue(pivotSubsys.setAngle(0));
      // joystick2.button(3).onTrue(pivotSubsys.setAngle(90));
      // joystick2.button(4).onTrue(pivotSubsys.setAngle(150));
    } else {

      


      xboxController.a().onTrue(drivetrainSubsys.driveToPose(getPose().plus(new Transform2d(new Translation2d(-1.0, -1.0), new Rotation2d(0)))));
      xboxController.x().onTrue(intakeSubsys.runIntake(0.2).until(() -> !intakeSubsys.sensorIsTriggered()).andThen(intakeSubsys.stopIntake()));
      xboxController.b().onTrue(pivotSubsys.setAngle(80));
      xboxController.y().onTrue(intakeSubsys.runIntake(-0.6));

      //xboxController.b().onTrue(superstructure.getCoral());
      // xboxController.y().onTrue(intakeSubsys.runIntake(3.0));
      // xboxController.y().onFalse(intakeSubsys.stopIntake());
      xboxController.rightBumper().onTrue(superstructure.getCoral());


      joystick.button(5).onTrue(superstructure.scoreL1Coral());
      joystick.button(6).onTrue(superstructure.scoreL2Coral());
      joystick.button(3).onTrue(superstructure.scoreL3Coral());
      joystick.button(4).onTrue(superstructure.scoreL4Coral());
      joystick.trigger().onTrue(superstructure.score());
      joystick.button(2).onTrue(superstructure.down());
      joystick.button(11).onTrue(superstructure.getCoral());
      //xboxController.y().onTrue(intakeSubsys.runIntake(-3.0)).onFalse(intakeSubsys.stopIntake());

    //bxController.x().onTrue(intakeSubsys.stopIntake());
      xboxController.leftTrigger().whileTrue(
        drivetrainSubsys.applyRequest(() -> drive.withVelocityX(0.1 *  polarityChooser.getSelected() * -xboxController.getLeftY() * MaxSpeed)

            .withVelocityY(0.1 * polarityChooser.getSelected() * -xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(0.1 * -xboxController.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative
                                                                              // X (left)

      // reset the field-centric heading on left bumper press
      xboxController.leftBumper().onTrue(drivetrainSubsys.runOnce(() -> drivetrainSubsys.seedFieldCentric()));

      drivetrainSubsys.registerTelemetry(logger::telemeterize);
    }
  }

  public void startPose() {
     drivetrainSubsys.resetPose(flyPathPosesConstants.startingPose);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configAutonomous() {
    SmartDashboard.putData(autoChooser);
  }

  public Pose2d getPose() {
    return drivetrainSubsys.getState().Pose;
  }

  public void resetSimulation() {
     vision.reset();
  }

  public void updateVision() {
     vision.update(getPose());
  }

  public Pose2d getNearestCoralScoringPose(Direction direction) {
    List<Pose2d> tagPoses = Collections.emptyList();

    int sideModifier = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? 0 : 11;

    for (int i = (22 - sideModifier); i <= (17 - sideModifier); i--) {
      tagPoses.add(VisionConstants.kFieldLayout.getTagPose(i).get().toPose2d());
    }

    Pose2d reefCenterPose = new Pose2d(4.026, ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? 4.489 : (kFieldLayout.getFieldLength() - 4.489)), new Rotation2d(0.0));
    Pose2d nearestPose = getPose().nearest(tagPoses);
    nearestPose = nearestPose.transformBy(nearestPose.minus(reefCenterPose));

    double initialRadius = 0.832;
    double finalRadius = initialRadius + 0.5;
    double modAngle = ((direction == Direction.Left) ? -1.0 : 1.0) * Math.atan(0.162 / finalRadius);

    return new Pose2d(
      ((finalRadius * Math.cos(Math.acos(nearestPose.getX() / initialRadius) + modAngle)) + reefCenterPose.getX()),
      ((finalRadius * Math.sin(Math.asin(nearestPose.getY() / initialRadius) + modAngle)) + reefCenterPose.getY()),
      new Rotation2d(-nearestPose.getRotation().getRadians())
    );
  }
}