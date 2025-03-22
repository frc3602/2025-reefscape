/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.Constants.DrivetrainConstants;
import frc.team3602.robot.Constants.ElevatorConstants;
import frc.team3602.robot.generated.TunerConstants;
import frc.team3602.robot.subsystems.ClimberSubsystem;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;

import static frc.team3602.robot.Constants.ElevatorConstants.pivotStowHeight;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.*;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric robocentricDrive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Operator Interfaces, Real */
  private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  private final CommandJoystick joystick = new CommandJoystick(kControlPanelPort);

  /* Operator Interfaces, Simulated */
  // private final CommandJoystick joystick = new CommandJoystick(0);

  /* Subsystems */
  private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
  private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem(
      elevatorSubsys.elevatorSimMech.getRoot("Pivot Root", 0.75, 0.7), () -> elevatorSubsys.elevatorViz.getLength());
  private final IntakeSubsystem intakeSubsys = new IntakeSubsystem(
      elevatorSubsys.elevatorSimMech.getRoot("Intake Wheel Root", 0.75, 0.3),
      () -> elevatorSubsys.elevatorViz.getLength(), () -> pivotSubsys.pivotSim.getAngleRads());
  private final ClimberSubsystem climberSubsys = new ClimberSubsystem();

  private final Vision vision = new Vision();
  private final Superstructure superstructure = new Superstructure(/* drivetrainSubsys, */ elevatorSubsys, intakeSubsys,
      pivotSubsys /* , vision */);

  /* Autonomous */
  private final SendableChooser<Command> autoChooser;
  private SendableChooser<Double> polarityChooser = new SendableChooser<>();

  // public RegionOfInterest getROI() {
  //   return (joystick.getRawAxis(3) >= 0.0) ? DrivetrainConstants.leftmostRegion : DrivetrainConstants.rightmostRegion;
  // }

  public LaserCan leftLASER = new LaserCan(kLeftLASERCANId);
  public LaserCan rightLASER = new LaserCan(kRightLASERCANId);

  public boolean LASERNotTriggered() {
    int LASERMeasureMM = ((joystick.getRawAxis(3) <= 0) ? rightLASER : leftLASER).getMeasurement().distance_mm;
    return ((LASERMeasureMM <= 0) || (LASERMeasureMM > 400));
  }

  public RobotContainer() {
    // Register commands for auton
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
      // drivetrainSubsys.applyRequest(() ->
      // drive.withVelocityX(-joystick.getRawAxis(0) * MaxSpeed) // Drive forward
      // // with negative Y
      // // (forward)
      // .withVelocityY(joystick.getRawAxis(1) * MaxSpeed) // Drive left with negative
      // X (left)
      // .withRotationalRate(-joystick2.getRawAxis(1) * MaxAngularRate)) // Drive
      // counterclockwise with negative X
      // // (left)
      // );
    } else {
      drivetrainSubsys.setDefaultCommand(
          drivetrainSubsys.applyRequest(
              () -> drive.withVelocityX(0.5 * polarityChooser.getSelected() * -xboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                  .withVelocityY(0.5 * polarityChooser.getSelected() * -xboxController.getLeftX() * MaxSpeed) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                  .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative
                                                                                // X (left)
          ));
    }
  }

  /*
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
      xboxController.leftTrigger().whileTrue(
          drivetrainSubsys.applyRequest(
              () -> drive.withVelocityX(0.1 * polarityChooser.getSelected() * -xboxController.getLeftY() * MaxSpeed)
                  .withVelocityY(0.1 * polarityChooser.getSelected() * -xboxController.getLeftX() * MaxSpeed) // Drive
                                                                                                              // left
                                                                                                              // with
                                                                                                              // negative
                                                                                                              // X
                                                                                                              // (left)
                  .withRotationalRate(0.3 * -xboxController.getRightX() * MaxAngularRate))); // Drive counterclockwise
                                                                                             // with negative
      xboxController.rightBumper().onTrue(superstructure.getCoral());
      xboxController.rightTrigger().whileTrue(
          drivetrainSubsys.applyRequest(() -> drivetrainSubsys.robocentricDrive
              .withVelocityX(0.1 * -xboxController.getLeftY() * MaxSpeed)
              .withVelocityY(0.1 * -xboxController.getLeftX() * MaxSpeed) // Drive left
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
              .withRotationalRate(0.3 * -xboxController.getRightX() * MaxAngularRate)) // Drive counterclockwise with
                                                                                       // negative
      );

      xboxController.povUp().onTrue(climberSubsys.runIn()).onFalse(climberSubsys.stop());
      xboxController.povDown().onTrue(climberSubsys.runOut()).onFalse(climberSubsys.stop());

      xboxController.a().whileTrue(drivetrainSubsys.align(() -> joystick.getRawAxis(3)).until(this::LASERNotTriggered).andThen(drivetrainSubsys.applyRequest(() -> brake)));
      // xboxController.a().onTrue(drivetrainSubsys.setROI(this::getROI)).whileTrue(drivetrainSubsys.align(() -> joystick.getRawAxis(3)).until(drivetrainSubsys::alignLASERIsTriggered).andThen(drivetrainSubsys.applyRequest(() -> brake)));
       xboxController.b().onTrue(pivotSubsys.setAngle(0.0));
      // xboxController.b().onTrue(drivetrainSubsys.runOnce(() -> drivetrainSubsys.resetPose(new Pose2d(0, 0, new Rotation2d(0.0)))));
      xboxController.x().onTrue(intakeSubsys.runIntake(0.2).until(intakeSubsys::sensorIsNotTriggered)
          .andThen(intakeSubsys.stopIntake()));
      xboxController.y().onTrue(intakeSubsys.runIntake(-0.6));
      // xboxController.y().onTrue(intakeSubsys.runIntake(3.0));
      // xboxController.y().onTrue(intakeSubsys.runIntake(-3.0)).onFalse(intakeSubsys.stopIntake());
      // xboxController.y().onFalse(intakeSubsys.stopIntake());

      joystick.button(5).onTrue(superstructure.scoreL1Coral());
      joystick.button(6).onTrue(superstructure.scoreL2Coral());
      joystick.button(3).onTrue(superstructure.scoreL3Coral());
      joystick.button(4).onTrue(superstructure.scoreL4Coral());
      joystick.trigger().onTrue(superstructure.score());
      joystick.button(2).onTrue(superstructure.down());

      joystick.button(11).onTrue(superstructure.grabAlgaeHigh());
      joystick.button(12).onTrue(superstructure.grabAlgaeLow());
      

      joystick.button(7).onTrue(superstructure.placeAlgaeInBarge());

      joystick.button(8).onTrue(superstructure.scoreAlgae());
      joystick.button(10).onTrue(superstructure.setAlgaeProcesser());
      joystick.button(9).onTrue(superstructure.stowAlgae());

      drivetrainSubsys.registerTelemetry(logger::telemeterize);

      // joystick.povUp().onTrue(elevatorSubsys.resetEncoder());
      // joystick.povLeft().onTrue(elevatorSubsys.runDown(true)).onFalse(elevatorSubsys.runDown(false));
      // joystick.povDown().onTrue(superstructure.downFromBarge());
      joystick.povUp().onTrue(climberSubsys.runIn()).onFalse(climberSubsys.stop());
      joystick.povDown().onTrue(climberSubsys.runOutAutomatic());
      joystick.povRight().onTrue(climberSubsys.runOut()).onFalse(climberSubsys.stop());
      joystick.povLeft().onTrue(superstructure.downFromBarge());
      joystick.axisLessThan(2, 0.0).onTrue(climberSubsys.resetEncoder());
    }
  }

  // public void startPose() {
  // drivetrainSubsys.resetPose(flyPathPosesConstants.startingPose);
  // }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void configAutonomous() {
    SmartDashboard.putData(autoChooser);
  }

  public void updatePose() {
    // puts the drivetrain pose on our dashboards
    SmartDashboard.putNumber("estimated drive pose x", drivetrainSubsys.getState().Pose.getX());
    SmartDashboard.putNumber("estimated drive pose y", drivetrainSubsys.getState().Pose.getY());
    SmartDashboard.putNumber("estimated drive pose rotation",
        drivetrainSubsys.getState().Pose.getRotation().getDegrees());

  
    //new and successful pose est
    
    try{
      var mod0VisionEst = vision.getMod0EstimatedPose();
      mod0VisionEst.ifPresent(est-> {
        drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        SmartDashboard.putNumber("mod 0 Cam Pose X", est.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("mod 0 Cam Pose Y", est.estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("mod 0 Cam Pose Angle",
            est.estimatedPose.toPose2d().getRotation().getDegrees());
      });
    } catch (Exception e0){
      Commands.print("Mod 0 pose est failed");
    }

    try{
      var mod1VisionEst = vision.getMod1EstimatedPose();
      mod1VisionEst.ifPresent(est-> {
        drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        SmartDashboard.putNumber("mod 1 Cam Pose X", est.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("mod 1 Cam Pose Y", est.estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("mod 1 Cam Pose Angle",
            est.estimatedPose.toPose2d().getRotation().getDegrees());
      });
    } catch (Exception e1){
      Commands.print("Mod 1 pose est failed");
    }

    try{
      var mod2VisionEst = vision.getMod2EstimatedPose();
      mod2VisionEst.ifPresent(est-> {
        drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        SmartDashboard.putNumber("mod 2 Cam Pose X", est.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("mod 2 Cam Pose Y", est.estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("mod 2 Cam Pose Angle",
            est.estimatedPose.toPose2d().getRotation().getDegrees());
      });
    } catch (Exception e2){
      Commands.print("Mod 2 pose est failed");
    }

    try{
      var mod3VisionEst = vision.getMod3EstimatedPose();
      mod3VisionEst.ifPresent(est-> {
        drivetrainSubsys.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        SmartDashboard.putNumber("mod 3 Cam Pose X", est.estimatedPose.toPose2d().getX());
        SmartDashboard.putNumber("mod 3 Cam Pose Y", est.estimatedPose.toPose2d().getY());
        SmartDashboard.putNumber("mod 3 Cam Pose Angle",
            est.estimatedPose.toPose2d().getRotation().getDegrees());
      });
    } catch (Exception e3){
      Commands.print("Mod 3 pose est failed");
    }





    // var mostRecentMod0Pose = new Pose2d();
    // try {
    //   var Mod0Pose = vision.getMod0EstimatedPose().get().estimatedPose.toPose2d();
    //   mostRecentMod0Pose = Mod0Pose;
    //   drivetrainSubsys.addVisionMeasurement(mostRecentMod0Pose, vision.lastMod0EstimateTimestamp, kMultiTagStdDevs);
    //   SmartDashboard.putNumber("mod 0 Cam Pose X", mostRecentMod0Pose.getX());
    //   SmartDashboard.putNumber("mod 0 Cam Pose Y", mostRecentMod0Pose.getY());
    //   SmartDashboard.putNumber("mod 0 Cam Pose Angle",
    //       mostRecentMod0Pose.getRotation().getDegrees());
    // } catch (Exception e) {
    //   Commands.print("mod 0 cam pose failed");
    // }

    // var mostRecentMod1Pose = new Pose2d();
    // try {
    //   var Mod1Pose = vision.getMod1EstimatedPose().get().estimatedPose.toPose2d();
    //   mostRecentMod1Pose = Mod1Pose;
    //   drivetrainSubsys.addVisionMeasurement(mostRecentMod1Pose, vision.lastMod1EstimateTimestamp, kMultiTagStdDevs);
    //   SmartDashboard.putNumber("mod 1 Cam Pose X", mostRecentMod1Pose.getX());
    //   SmartDashboard.putNumber("mod 1 Cam Pose Y", mostRecentMod1Pose.getY());
    //   SmartDashboard.putNumber("mod 1 Cam Pose Angle",
    //       mostRecentMod1Pose.getRotation().getDegrees());
    // } catch (Exception e) {
    //   Commands.print("mod 1 cam pose failed");
    // }

    // var mostRecentMod2Pose = new Pose2d();
    // try {
    //   var Mod2Pose = vision.getMod2EstimatedPose().get().estimatedPose.toPose2d();
    //   mostRecentMod1Pose = Mod2Pose;
    //   drivetrainSubsys.addVisionMeasurement(mostRecentMod2Pose, vision.lastMod2EstimateTimestamp, kMultiTagStdDevs);
    //   SmartDashboard.putNumber("mod 2 Cam Pose X", mostRecentMod2Pose.getX());
    //   SmartDashboard.putNumber("mod 2 Cam Pose Y", mostRecentMod2Pose.getY());
    //   SmartDashboard.putNumber("mod 2 Cam Pose Angle",
    //       mostRecentMod2Pose.getRotation().getDegrees());
    // } catch (Exception e) {
    //   Commands.print("mod 2 cam pose failed");
    // }

    // var mostRecentMod3Pose = new Pose2d();
    // try {
    //   var Mod3Pose = vision.getMod3EstimatedPose().get().estimatedPose.toPose2d();
    //   mostRecentMod3Pose = Mod3Pose;
    //   drivetrainSubsys.addVisionMeasurement(mostRecentMod3Pose, vision.lastMod3EstimateTimestamp, kMultiTagStdDevs);
    //   SmartDashboard.putNumber("mod 3 Cam Pose X", mostRecentMod3Pose.getX());
    //   SmartDashboard.putNumber("mod 3 Cam Pose Y", mostRecentMod3Pose.getY());
    //   SmartDashboard.putNumber("mod 3 Cam Pose Angle",
    //       mostRecentMod2Pose.getRotation().getDegrees());
    // } catch (Exception e) {
    //   Commands.print("mod 3 cam pose failed");
    // }
    // allows us to reset our pose
    // TODO take out for matches
    // for auton testing at LSSU, change the pose to the starting pose we have in
    // pathplanner. Note that the rotation2d value is found in the starting state,
    // NOT the start pose
    if (joystick.button(5).getAsBoolean()) {
      drivetrainSubsys.resetPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    var newestPose = drivetrainSubsys.getState().Pose;
    drivetrainSubsys.resetPose(newestPose);
  }

  public void updateSimulation() {
    vision.visionSim.update(drivetrainSubsys.getState().Pose);
    vision.visionSim.getDebugField();
  }

  public void resetSimulation() {
    vision.reset();
  }

}