/*
 * Copyright (C) 2025, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.team3602.robot.Constants.ElevatorConstants;
import frc.team3602.robot.Constants.IntakeConstants;
import frc.team3602.robot.Constants.PivotConstants;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;

public class Superstructure extends SubsystemBase{
    private DrivetrainSubsystem driveSubsys;
    private ElevatorSubsystem elevatorSubsys;
    private IntakeSubsystem intakeSubsys;
    private PivotSubsystem pivotSubsys;
    private Vision vision;

    public double mostRecentElevatorHeight = 0.1;
    public double mostRecentPivotAngle = 0;

    public Superstructure(DrivetrainSubsystem driveSubsys, ElevatorSubsystem elevatorSubsys, IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, Vision vision) {
        this.driveSubsys = driveSubsys;
        this.elevatorSubsys = elevatorSubsys;
        this.intakeSubsys = intakeSubsys;
        this.pivotSubsys = pivotSubsys;
        this.vision = vision;
    }

    public Command getCoral() {
        return Commands.sequence(
            pivotSubsys.setAngle(PivotConstants.highStowAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal()), 

            elevatorSubsys.setHeight(0.0),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),

            pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal()),

            intakeSubsys.runIntake(0.3),
            Commands.waitUntil(() -> intakeSubsys.sensorIsTriggered()),
            intakeSubsys.runIntake(-0.5).withTimeout(1),
            intakeSubsys.stopIntake()
        );
   }

    public Command scoreL4CoralCommand() {
        return Commands.sequence(
            Commands.print("start seq"),
            intakeSubsys.stopIntake(),
            pivotSubsys.setAngle(PivotConstants.lowStowAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
            Commands.print("pivot in stow angle"),
            elevatorSubsys.setHeight(ElevatorConstants.scoreLevelFour),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            Commands.print("elev in place"),
            pivotSubsys.setAngle(PivotConstants.scoreL4Angle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal())

        );
    }

    public Command scoreL3CoralCommand() {
        return Commands.sequence(
            intakeSubsys.stopIntake(),
            Commands.print("start seq"),
            pivotSubsys.setAngle(PivotConstants.highStowAngle),
            Commands.waitUntil(() -> pivotSubsys.isStowed()),
            Commands.print("pivot in stow angle"),
            elevatorSubsys.setHeight(ElevatorConstants.scoreLevelThree),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            Commands.print("elev in place"),
            pivotSubsys.setAngle(PivotConstants.scoreCoralAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal())
            // intakeSubsys.runIntake(IntakeConstants.coralSpeed),
            // Commands.waitUntil(() -> !intakeSubsys.sensorIsTriggered()),
            // intakeSubsys.stopIntake()
        );
    }
    public Command scoreL2CoralCommand() {
        return Commands.sequence(
            intakeSubsys.stopIntake(),
            Commands.print("start seq"),
            pivotSubsys.setAngle(PivotConstants.highStowAngle),
            Commands.waitUntil(() -> pivotSubsys.isStowed()),
            Commands.print("pivot in stow angle"),
            elevatorSubsys.setHeight(ElevatorConstants.scoreLevelTwo),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            Commands.print("elev in place"),
            pivotSubsys.setAngle(PivotConstants.scoreCoralAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal())
            // intakeSubsys.runIntake(IntakeConstants.coralSpeed),
            // Commands.waitUntil(() -> !intakeSubsys.sensorIsTriggered()),
            // intakeSubsys.stopIntake()
        );
    }

    public Command scoreL1CoralCommand() {
        return Commands.sequence(
            Commands.print("start seq"),
            intakeSubsys.stopIntake(),
            pivotSubsys.setAngle(PivotConstants.lowStowAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
            Commands.print("pivot in stow angle"),
            elevatorSubsys.setHeight(ElevatorConstants.scoreLevelOne),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            Commands.print("elev in place"),
            pivotSubsys.setAngle(PivotConstants.scoreCoralAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoal())
        //     intakeSubsys.runIntake(0.5),
        //     Commands.waitUntil(() -> !intakeSubsys.sensorIsTriggered()),
        //     intakeSubsys.stopIntake()
         );
    }

    public Command score(){
        return Commands.sequence(
        intakeSubsys.runIntake(IntakeConstants.coralSpeed),
        Commands.waitUntil(() -> ! intakeSubsys.sensorIsTriggered()),
        intakeSubsys.stopIntake(),

        pivotSubsys.setAngle(PivotConstants.lowStowAngle),
        Commands.waitUntil(()-> pivotSubsys.isNearGoal()),

        elevatorSubsys.setHeight(0.1),

        pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
        Commands.waitUntil(() -> pivotSubsys.isNearGoal()),


        intakeSubsys.runIntake(0.3),
        Commands.waitUntil(() -> intakeSubsys.sensorIsTriggered()),
        intakeSubsys.stopIntake()
        );
    }

    public Command ridIntakeOfCoral() {
        return Commands.sequence(
            intakeSubsys.runIntake(IntakeConstants.coralSpeed),
            Commands.waitUntil(() -> ! intakeSubsys.sensorIsTriggered()),
            intakeSubsys.stopIntake()
        );
    }
}