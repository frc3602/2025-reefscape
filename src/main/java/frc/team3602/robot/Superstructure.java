/*
 * Copyright (C) 2025, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.GripperSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;

public class Superstructure {
        private final DrivetrainSubsystem drivetrainSubsys;
        private final ElevatorSubsystem elevatorSubsys;
        private final PivotSubsystem pivotSubsys;
        private final GripperSubsystem gripperSubsys;
 
    public Superstructure(DrivetrainSubsystem drivetrainSubsys, ElevatorSubsystem elevatorSubsys, PivotSubsystem pivotSubsys, GripperSubsystem gripperSubsys) {
        this.drivetrainSubsys = drivetrainSubsys;
        this.elevatorSubsys = elevatorSubsys;
        this.pivotSubsys = pivotSubsys;
        this.gripperSubsys = gripperSubsys;
    }
}