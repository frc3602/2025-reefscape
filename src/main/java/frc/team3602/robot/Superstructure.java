/*
 * Copyright (C) 2025, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.team3602.robot.Constants.ElevatorConstants;
import frc.team3602.robot.Constants.PivotConstants;
import frc.team3602.robot.scoring.CoralScoreDescriptor;
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

    public double mostRecentElevatorHeight;

    public Superstructure(DrivetrainSubsystem driveSubsys, ElevatorSubsystem elevatorSubsys, IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, Vision vision) {
        this.driveSubsys = driveSubsys;
        this.elevatorSubsys = elevatorSubsys;
        this.intakeSubsys = intakeSubsys;
        this.pivotSubsys = pivotSubsys;
        this.vision = vision;
    }

    public Command scoreCoral(CoralScoreDescriptor scoreDescriptor) {
        return Commands.sequence(
            // TODO: Navigate Robot scoreDescriptor.direction (Left or Right).
            ((scoreDescriptor.getElevatorHeight() > ElevatorConstants.pivotStowHeight) ? Commands.sequence(
                pivotSubsys.setAngle(PivotConstants.stowAngle),
                Commands.print("Stowing Pivot"),
                Commands.waitUntil(() -> pivotSubsys.isNearGoalAngle())   
            ) : Commands.none()),
            elevatorSubsys.setHeight(scoreDescriptor.getElevatorHeight()),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            Commands.print("Elevator Placed"),
            pivotSubsys.setAngle(scoreDescriptor.getPivotAngle()),
            Commands.waitUntil(() -> pivotSubsys.isNearGoalAngle()),
            Commands.print("Pivot Angled")//,
            // intakeSubsys.runIntake(3.0),
            // Commands.none().until(() -> true),
            // // TODO: Replace () -> true with laser CAN
            // intakeSubsys.stopIntake(),
            // ((scoreDescriptor.getElevatorHeight() > ElevatorConstants.pivotStowHeight) ? Commands.sequence(
            //     pivotSubsys.setAngle(PivotConstants.stowAngle),
            //     Commands.print("Stowing Pivot"),
            //     Commands.waitUntil(() -> pivotSubsys.isNearGoalAngle())   
            // ) : Commands.none()),
            // elevatorSubsys.setHeight(0.0),
            // Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            // pivotSubsys.setAngle(PivotConstants.coralIntakeAngle)
        );
   }

    public Command scoreL4CoralCommand() {
        return Commands.sequence(
            Commands.print("start seq"),
            pivotSubsys.setAngle(PivotConstants.stowAngle),
            Commands.waitUntil(() -> pivotSubsys.isNearGoalAngle()),
            Commands.print("pivot in stow angle"),
            elevatorSubsys.setHeight(ElevatorConstants.scoreLevelFour),
            Commands.print("elev in place"),
            Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
            pivotSubsys.setAngle(PivotConstants.scoreL4Angle)
        );
    }

}