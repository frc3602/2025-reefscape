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
import frc.team3602.robot.subsystems.WaitableSubsystem;

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

    public Command waitOn(WaitableSubsystem subsystem) {
        return Commands.none().until(() -> subsystem.isNearGoal());
    }

    public Command scoreCoral(CoralScoreDescriptor scoreDescriptor) {
        return Commands.sequence(
            // TODO: Navigate Robot scoreDescriptor.direction (Left or Right).
            ((scoreDescriptor.getElevatorHeight() > ElevatorConstants.pivotStowHeight) ? Commands.sequence(
                pivotSubsys.stowPivot(),
                Commands.print("Stowing Pivot"),
                waitOn(pivotSubsys)   
            ) : Commands.none()),
            elevatorSubsys.setHeight(scoreDescriptor.getElevatorHeight()),
            waitOn(elevatorSubsys),
            Commands.print("Elevator Placed"),
            pivotSubsys.setAngle(scoreDescriptor.getPivotAngle()),
            waitOn(pivotSubsys),
            Commands.print("Pivot Angled"),
            intakeSubsys.runIntake(3.0),
            Commands.none().until(() -> true),
            // TODO: Replace () -> true with laser CAN
            intakeSubsys.stopIntake(),
            ((scoreDescriptor.getElevatorHeight() > ElevatorConstants.pivotStowHeight) ? Commands.sequence(
                pivotSubsys.stowPivot(),
                Commands.print("Stowing Pivot"),
                waitOn(pivotSubsys)   
            ) : Commands.none()),
            elevatorSubsys.setHeight(0.0),
            waitOn(elevatorSubsys),
            pivotSubsys.setAngle(PivotConstants.coralIntakeAngle)
        );
    }

    public Command scoreL4CoralCommand() {
        return Commands.sequence(
            Commands.print("start seq"),
            pivotSubsys.stowPivot(),
            waitOn(pivotSubsys),
            Commands.print("pivot in stow angle"),
            elevatorSubsys.setHeight(ElevatorConstants.scoreLevelFour),
            Commands.print("elev in place"),
            waitOn(elevatorSubsys),
            pivotSubsys.setAngle(PivotConstants.scoreL4Angle)
        );
    }

}