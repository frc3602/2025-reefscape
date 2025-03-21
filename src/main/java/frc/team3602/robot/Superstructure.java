/*
 * Copyright (C) 2025, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import static frc.team3602.robot.Constants.ElevatorConstants.*;
import static frc.team3602.robot.Constants.IntakeConstants.*;
import static frc.team3602.robot.Constants.PivotConstants.*;

import frc.team3602.robot.Constants.ElevatorConstants;
import frc.team3602.robot.Constants.PivotConstants;
// import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.ElevatorSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;

public class Superstructure extends SubsystemBase {
    // private DrivetrainSubsystem driveSubsys;
    private ElevatorSubsystem elevatorSubsys;
    private IntakeSubsystem intakeSubsys;
    private PivotSubsystem pivotSubsys;
    // private Vision vision;

    public Superstructure(/* DrivetrainSubsystem driveSubsys, */ ElevatorSubsystem elevatorSubsys,
            IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys /* , Vision vision */) {
        // this.driveSubsys = driveSubsys;
        this.elevatorSubsys = elevatorSubsys;
        this.intakeSubsys = intakeSubsys;
        this.pivotSubsys = pivotSubsys;
        // this.vision = vision;
    }

    public Command getCoral() {
        return sequence(
                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(0.0),
                waitUntil(elevatorSubsys::isNearGoal),

                pivotSubsys.setAngle(coralIntakeAngle),
                waitUntil(pivotSubsys::isNearGoal),

                intakeSubsys.runIntake(0.3),
                waitUntil(intakeSubsys::sensorIsTriggered),
                intakeSubsys.runIntake(-0.5).withTimeout(1.0),
                intakeSubsys.stopIntake());
    }

    public Command scoreL1Coral() {
        return sequence(
                print("Start of Sequence"),

                intakeSubsys.stopIntake(),

                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Stowed"),

                elevatorSubsys.setHeight(scoreLevelOne),
                waitUntil(elevatorSubsys::isNearGoal),
                print("Elevator Positioned"),

                pivotSubsys.setAngle(scoreCoralAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Positioned")
        ).unless(intakeSubsys::sensorIsNotTriggered);
    }

    public Command scoreL2Coral() {
        return sequence(
                print("Start of Sequence"),

                intakeSubsys.stopIntake(),

                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Stowed"),

                elevatorSubsys.setHeight(scoreLevelTwo),
                waitUntil(elevatorSubsys::isNearGoal),
                print("Elevator Positioned"),

                pivotSubsys.setAngle(scoreCoralAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Positioned")
        ).unless(intakeSubsys::sensorIsNotTriggered);
    }

    public Command scoreL3Coral() {
        return sequence(
                print("Start of Sequence"),

                intakeSubsys.stopIntake(),

                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Stowed"),

                elevatorSubsys.setHeight(scoreLevelThree),
                waitUntil(elevatorSubsys::isNearGoal),
                print("Elevator Positioned"),

                pivotSubsys.setAngle(scoreCoralAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Positioned")
        ).unless(intakeSubsys::sensorIsNotTriggered);
    }

    public Command scoreL4Coral() {
        return sequence(
                print("Start of Sequence"),

                intakeSubsys.stopIntake(),
                pivotSubsys.setAngle(PivotConstants.l4StowAngle),
                waitUntil(pivotSubsys::isNearGoal),
                elevatorSubsys.setHeight(scoreLevelThree),

                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),
                print("Pivot Stowed"),

                elevatorSubsys.setHeight(scoreLevelFour),
                // waitUntil(() -> elevatorSubsys.isAbove(15)),
                print("elev above the point of no return!!!"),

                pivotSubsys.setAngle(scoreL4Angle),
                waitUntil(pivotSubsys::isNearGoal)
        ).unless(intakeSubsys::sensorIsNotTriggered);
    }

    public Command score() {
        return intakeSubsys.runIntake(coralSpeed).until(() -> !intakeSubsys.sensorIsTriggered());
    }

    public Command down() {
        return Commands.sequence(
            sequence(
                elevatorSubsys.setHeight(ElevatorConstants.scoreLevelThree),
                waitUntil(elevatorSubsys::isNearGoal)
            ).unless(elevatorSubsys::beneathL4Height),

            pivotSubsys.setAngle(stowAngle),
            waitUntil(pivotSubsys::isNearGoal),

            elevatorSubsys.setHeight(0.1),

            pivotSubsys.setAngle(coralIntakeAngle),
            waitUntil(pivotSubsys::isNearGoal),
            intakeSubsys.runIntake(0.1).until(intakeSubsys::sensorIsTriggered)
        );
    }

    public Command downFromBarge() {
        return sequence(
            pivotSubsys.setAngle(20.0),
            waitUntil(pivotSubsys::isNearGoal),

            elevatorSubsys.setHeight(0.1),
            waitUntil(elevatorSubsys::isNearGoal)
        );
    }

    public Command grabAlgaeHigh() {
        return sequence(
                pivotSubsys.setAngle(0.0),
                waitUntil(pivotSubsys::isStowed),

                elevatorSubsys.setHeight(removeAlgaeHigh),

                pivotSubsys.setAngle(-50.0),

                intakeSubsys.holdAlgae(intakeAlgaeSpeed));
    }

    public Command grabAlgaeLow() {
        return sequence(
                pivotSubsys.setAngle(0),
                waitUntil(pivotSubsys::isStowed),

                elevatorSubsys.setHeight(removeAlgaeLow),

                pivotSubsys.setAngle(-50),

                intakeSubsys.runIntake(intakeAlgaeSpeed));
    }

    public Command holdAlgae() {
    return sequence(
                elevatorSubsys.setHeight(.5),
                pivotSubsys.setAngle(29)
                );
       
    }

    public Command setAlgaeProcesser() {
        return sequence(
            intakeSubsys.stopIntake(),
            parallel(
                elevatorSubsys.setHeight(scoreAlgaeProcesser),

                pivotSubsys.disablePivot()
            ),
            waitSeconds(2.0),
            pivotSubsys.enablePivot()
        );
    }


    // public Command stowAlgae() {
    //     return Commands.sequence(//parallel(
    //             intakeSubsys.runIntake(intakeAlgaeSpeed),

    //            // sequence(
    //                     elevatorSubsys.setHeight(0.1),

    //                     pivotSubsys.setAngle(29)//)
    //                     );
    // }

    public Command stowAlgae() {
        return sequence(
            
            intakeSubsys.setIntake(intakeAlgaeSpeed),

            sequence(
                elevatorSubsys.setHeight(0.1),

                pivotSubsys.setAngle(29)
            )
        );
    }

    public Command placeAlgaeInBarge() {
        return Commands.sequence(
            intakeSubsys.stopIntake(),

            pivotSubsys.setAngle(22.0),
            waitUntil(pivotSubsys::isNearGoal),

            elevatorSubsys.setHeight(scoreLevelThree),
            waitUntil(elevatorSubsys::isNearGoal),

            parallel(
                sequence(
                    elevatorSubsys.setHeight(scoreLevelFour),
                    waitUntil(elevatorSubsys::isNearGoal)
                ),
                sequence(
                    intakeSubsys.runIntake(0.5),
                    pivotSubsys.setAngle(30.0)
                )
            ),
            intakeSubsys.stopIntake()

            // parallel(
            //     sequence(
            //         elevatorSubsys.setHeight(Units.metersToInches(kMaxHeightMeters) - 0.5),
            //         waitUntil(elevatorSubsys::isNearGoal)
            //     ),
            //     sequence(
            //         waitUntil(elevatorSubsys::aboveShootingHeight),
            //         pivotSubsys.setAngle(36)
            //     )
            // )
        );
    }

    public Command scoreAlgae() {
        return intakeSubsys.runIntake(scorAlgeaSpeed);
    }

    // private boolean algaeTaken = false;

    public Command intakeAlgae() {
        // if (!algaeTaken) {
        // algaeTaken = true;
        if (elevatorSubsys.getEncoder() < 0.5) {
            return sequence(
                    elevatorSubsys.setHeight(0.1),

                    pivotSubsys.setAngle(intakeAlgaeAngle),

                    intakeSubsys.runIntake(intakeAlgaeSpeed));
        } else {
            // algaeTaken = false;
            return sequence(
                    pivotSubsys.setAngle(stowAngle),
                    waitUntil(pivotSubsys::isNearGoal),

                    elevatorSubsys.setHeight(0.1),
                    waitUntil(elevatorSubsys::isNearGoal),

                    pivotSubsys.setAngle(intakeAlgaeAngle),

                    intakeSubsys.runIntake(intakeAlgaeSpeed));
        }
        // } // ?
    }

    // private boolean algaeDropped = false;

    public Command dropAlgae() {
        // algaeDropped = !algaeDropped;

        // if (algaeDropped) {
        return sequence(
                intakeSubsys.runIntake(0.4),
                waitSeconds(1),

                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(0.1),

                intakeSubsys.runIntake(0.17).until(intakeSubsys::sensorIsTriggered),

                pivotSubsys.setAngle(coralIntakeAngle),
                waitUntil(pivotSubsys::isNearGoal));
        // } else {
        // return intakeSubsys.stopIntake();
        // }
    }

    public Command ridIntakeOfCoral() {
        return sequence(
                intakeSubsys.runIntake(coralSpeed),
                waitUntil(() -> !intakeSubsys.sensorIsTriggered()));
    }

    /* Commands for Autonomous */
    public Command autonPrepElevL1() {
        return sequence(
                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(scoreLevelOne),
                waitUntil(elevatorSubsys::isNearGoal));
    }

    public Command autonPrepElevL2() {
        return sequence(
                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(scoreLevelTwo),
                waitUntil(elevatorSubsys::isNearGoal));
    }

    public Command autonPrepElevL3() {
        return sequence(
                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(scoreLevelThree),
                waitUntil(elevatorSubsys::isNearGoal));
    }

    public Command autonPrepElevL4() {
        return sequence(
                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(31.0) // scoreLevelFour
        );
    }

    public Command autonPrepElevCoralIntake() {
        return sequence(
                pivotSubsys.setAngle(stowAngle),
                waitUntil(pivotSubsys::isNearGoal),

                elevatorSubsys.setHeight(0.1));
    }

    public Command autonPrepPivotReef() {
        return sequence(
                pivotSubsys.setAngle(scoreCoralAngle),
                waitUntil(pivotSubsys::isNearGoal));
    }

    public Command autonPrepPivotL4() {
        return sequence(
                pivotSubsys.setAngle(88.0), // scoreL4Angle
                waitUntil(pivotSubsys::isNearGoal));
    }

    public Command autonPrepPivotCoralIntake() {
        return sequence(
                pivotSubsys.setAngle(coralIntakeAngle),
                waitUntil(pivotSubsys::isNearGoal)
        // intakeSubsys.runIntake(0.2) // .until(intakeSubsys::sensorIsTriggered)
        );
    }

    public Command autonPrepPivotAlgae() {
        return sequence(
                pivotSubsys.setAngle(0),
                waitUntil(pivotSubsys::isStowed));
    }

    public Command autonShoot() {
        return intakeSubsys.runIntake(0.5).until(() -> !intakeSubsys.sensorIsTriggered());
    }

    public Command autonIntake() {
        return intakeSubsys.runIntake(0.1).until(intakeSubsys::sensorIsTriggered);
    }

    public Command autonGrabAlgaeHigh() {
        return sequence(
                // pivotSubsys.setAngle(0.0),
                // waitUntil(pivotSubsys::isStowed),

                elevatorSubsys.setHeight(removeAlgaeHigh),

                pivotSubsys.setAngle(-50.0),

                intakeSubsys.runIntake(intakeAlgaeSpeed));
    }

    public Command autonGrabAlgaeLow() {
        return sequence(
                // pivotSubsys.setAngle(0),
                // waitUntil(pivotSubsys::isStowed),

                elevatorSubsys.setHeight(removeAlgaeLow),

                pivotSubsys.setAngle(-50.0),

                intakeSubsys.runIntake(intakeAlgaeSpeed));
    }

    public Command autonHoldAlgae() {
        return sequence(
                intakeSubsys.runIntake(-0.6),

                elevatorSubsys.setHeight(0.1));
    }
}