/*
 * Copyright (C) 2025, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;

public class Superstructure extends SubsystemBase{
    private DrivetrainSubsystem driveSubsys;
    // private ElevatorSubsystem elevatorSubsys;
    private IntakeSubsystem intakeSubsys;
    private PivotSubsystem pivotSubsys;
    private Vision vision;

    public double mostRecentElevatorHeight;

    public Superstructure(DrivetrainSubsystem driveSubsys, ElevatorSubsystem elevatorSubsys,  IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, Vision vision){
        this.driveSubsys = driveSubsys;
        // this.elevatorSubsys = elevatorSubsys;
        this.intakeSubsys = intakeSubsys;
        this.pivotSubsys = pivotSubsys;
        this.vision = vision;
    }

    public Command intakeCoral(){
        return runEnd(() -> {
            intakeSubsys.runIntake(1.5).until(() -> intakeSubsys.sensorTriggered());
        }, () -> {
            intakeSubsys.stopIntake();
        });
    }  
    
    public Command outputCoral(){
        return runEnd(() -> {
            intakeSubsys.runIntake(1.5).until(() -> ! intakeSubsys.sensorTriggered());
            Commands.none().withTimeout(1);
        }, () -> {
            intakeSubsys.stopIntake();
        });
    }




}
