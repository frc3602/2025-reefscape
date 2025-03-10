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
 
 public class Superstructure extends SubsystemBase {
     private DrivetrainSubsystem driveSubsys;
     private ElevatorSubsystem elevatorSubsys;
     private IntakeSubsystem intakeSubsys;
     private PivotSubsystem pivotSubsys;
 
     public Superstructure(DrivetrainSubsystem driveSubsys, ElevatorSubsystem elevatorSubsys, IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys) {
         this.driveSubsys = driveSubsys;
         this.elevatorSubsys = elevatorSubsys;
         this.intakeSubsys = intakeSubsys;
         this.pivotSubsys = pivotSubsys;
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
 
     public Command scoreL4Coral() {
         return Commands.sequence(
             Commands.print("start seq"),
             intakeSubsys.stopIntake(),
             pivotSubsys.setAngle(PivotConstants.highStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
             Commands.print("pivot in stow angle"),
             elevatorSubsys.setHeight(ElevatorConstants.scoreLevelFour),
             // scoreLevelFour),
             // Commands.waitUntil(() -> elevatorSubsys.isAbove(15)),
             Commands.print("elev above the point of no return!!!"),
             pivotSubsys.setAngle(PivotConstants.scoreL4Angle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal())
 
         );
     }
 
     public Command scoreL3Coral() {
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
     public Command scoreL2Coral() {
         return Commands.sequence(
             intakeSubsys.stopIntake(),
             Commands.print("start seq"),
             pivotSubsys.setAngle(PivotConstants.lowStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
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
 
     public Command scoreL1Coral() {
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
 
         // if(elevatorSubsys.getEncoder() <0.5){
             return Commands.sequence(
                 intakeSubsys.runIntake(IntakeConstants.coralSpeed).until(() -> ! intakeSubsys.sensorIsTriggered())
 
         //     elevatorSubsys.setHeight(0.1),
 
         //     pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
         //     Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
     
     
         //     intakeSubsys.runIntake(0.17).until(() -> intakeSubsys.sensorIsTriggered())
      
         //     );  }else{
 
           
         // return Commands.sequence(
 
         //     pivotSubsys.setAngle(PivotConstants.highStowAngle),
         //     Commands.waitUntil(()-> pivotSubsys.isNearGoal()),
 
         //     Commands.waitSeconds(3),
         //     elevatorSubsys.setHeight(0.1),
 
         //     pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
         //     Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
 
 
         //     intakeSubsys.runIntake(0.17).until(() -> intakeSubsys.sensorIsTriggered())
    
         );
         
          }
     
 
     public Command down() {
         // if(elevatorSubsys.getEncoder() < 0.5) {
         //     return Commands.sequence(
         //         elevatorSubsys.setHeight(0.1),
         //         pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
         //         Commands.waitUntil(() -> pivotSubsys.isNearGoal())
         //     );
         // } else {
             return Commands.sequence(

                 pivotSubsys.setAngle(PivotConstants.lowStowAngle),
                 Commands.waitUntil(()-> pivotSubsys.isNearGoal()),
 
                 elevatorSubsys.setHeight(0.1),
 
                 pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
                 Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
                 intakeSubsys.runIntake(0.1).until(() -> intakeSubsys.sensorIsTriggered())
 
 
             );
         }
     
 
     public Command grabAlgaeHigh(){
         return Commands.sequence(
             pivotSubsys.setAngle(0),
             Commands.waitUntil(() -> pivotSubsys.isStowed()),
 
             elevatorSubsys.setHeight(ElevatorConstants.removeAlgaeHigh),
             pivotSubsys.setAngle(-50),
             intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed)
         );
     }
     public Command grabAlgaeLow(){
         return Commands.sequence(
             pivotSubsys.setAngle(0),
             Commands.waitUntil(() -> pivotSubsys.isStowed()),
 
             elevatorSubsys.setHeight(ElevatorConstants.removeAlgaeLow),
             pivotSubsys.setAngle(-50),
             intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed)
         );
     }
 
     public Command holdAlgae(){
         return Commands.sequence(
             intakeSubsys.runIntake(-0.6),
             elevatorSubsys.setHeight(0.1)
         );
     }
     
     public Command setAlgeaProcesser(){
        return Commands.parallel(
            intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed),
         Commands.sequence(

            elevatorSubsys.setHeight(ElevatorConstants.scoreAlgaeProcesser),
            pivotSubsys.setAngle(PivotConstants.scoreAlgaeProcesserAngle)
        )
        );
     }

     public Command scoreAlgea(){
        return Commands.sequence(
        intakeSubsys.runIntake(IntakeConstants.scorAlgeaSpeed)
        );
     }
    // private boolean algaeTaken = false;
 
     public Command intakeAlgae() {
         // if (!algaeTaken) {
         //     algaeTaken = true;
         if(elevatorSubsys.getEncoder() < 0.5){
 
             return Commands.sequence(
              
                 elevatorSubsys.setHeight(0.1),
 
                 pivotSubsys.setAngle(PivotConstants.intakeAlgaeAngle),
                 intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed)
             );
        } else {
             //algaeTaken = false;
             return Commands.sequence(
                 pivotSubsys.setAngle(PivotConstants.lowStowAngle),
                 Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
                 elevatorSubsys.setHeight(0.1),
                 Commands.waitUntil(() -> elevatorSubsys.isNearGoal()),
 
                 pivotSubsys.setAngle(PivotConstants.intakeAlgaeAngle),
                 intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed)
             );
           
         }
     }
 
     private boolean algaeDropped = false;
 
     public Command dropAlgae() {
         //algaeDropped = !algaeDropped;
 
        // if (algaeDropped) {
             return Commands.sequence(
                 intakeSubsys.runIntake(0.4),
                 Commands.waitSeconds(1),
 
                 pivotSubsys.setAngle(PivotConstants.lowStowAngle),
                 Commands.waitUntil(()-> pivotSubsys.isNearGoal()),
   
                 elevatorSubsys.setHeight(0.1),
   
                 intakeSubsys.runIntake(0.17).until(() -> intakeSubsys.sensorIsTriggered()),
                 pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
                  Commands.waitUntil(() -> pivotSubsys.isNearGoal())
             );
         // } else {
         //     return intakeSubsys.stopIntake();
         // }
     }
 
     public Command ridIntakeOfCoral() {
         return Commands.sequence(
             intakeSubsys.runIntake(IntakeConstants.coralSpeed),
             Commands.waitUntil(() -> !intakeSubsys.sensorIsTriggered())   
             );
     }
 
 
 
 
     //AUTONS COMMANDS
     public Command autonPrepElevL4() {
         return Commands.sequence(
             pivotSubsys.setAngle(PivotConstants.lowStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
             elevatorSubsys.setHeight(31.0)//ElevatorConstants.scoreLevelFour)
         );
     }
 
     public Command autonPrepElevL3() {
         return Commands.sequence(
             pivotSubsys.setAngle(PivotConstants.highStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
             elevatorSubsys.setHeight(ElevatorConstants.scoreLevelThree),
             Commands.waitUntil(() -> elevatorSubsys.isNearGoal())
         );
     }
 
 
     public Command autonPrepElevL2() {
         return Commands.sequence(
             pivotSubsys.setAngle(PivotConstants.lowStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
             elevatorSubsys.setHeight(ElevatorConstants.scoreLevelTwo),
             Commands.waitUntil(() -> elevatorSubsys.isNearGoal())
         );
     }
 
     public Command autonPrepElevL1() {
         return Commands.sequence(
             pivotSubsys.setAngle(PivotConstants.lowStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
             elevatorSubsys.setHeight(ElevatorConstants.scoreLevelOne),
             Commands.waitUntil(() -> elevatorSubsys.isNearGoal())
         );
     }
 
     public Command autonPrepElevCoralIntake(){
         return Commands.sequence(
             pivotSubsys.setAngle(PivotConstants.lowStowAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal()),
             elevatorSubsys.setHeight(0.1)
         );
     } 
 
 
     public Command autonPrepPivotReef(){
         return Commands.sequence(
         pivotSubsys.setAngle(PivotConstants.scoreCoralAngle),
         Commands.waitUntil(() -> pivotSubsys.isNearGoal())
         );
     } 
 
     public Command autonPrepPivotL4(){
         return Commands.sequence(
         pivotSubsys.setAngle(88),//PivotConstants.scoreL4Angle),
         Commands.waitUntil(() -> pivotSubsys.isNearGoal())
         );
     }
 
     public Command autonPrepPivotCoralIntake(){
         return Commands.sequence(
             pivotSubsys.setAngle(PivotConstants.coralIntakeAngle),
             Commands.waitUntil(() -> pivotSubsys.isNearGoal())
             //intakeSubsys.runIntake(0.2)//.until(() -> intakeSubsys.sensorIsTriggered())
             );
     }
 
     public Command autonPrepPivotAlgae(){
         return Commands.sequence(           
          pivotSubsys.setAngle(0),
         Commands.waitUntil(() -> pivotSubsys.isStowed())
         );
     }
 

     public Command autonShoot(){
         return Commands.sequence(
             intakeSubsys.runIntake(1.0).until(() -> !intakeSubsys.sensorIsTriggered())
         );
     }
 
     public Command autonIntake(){
         return Commands.sequence(
             intakeSubsys.runIntake(0.2).until(() -> intakeSubsys.sensorIsTriggered())
         );
     }
 
 
 
 
     public Command autonGrabAlgaeHigh(){
         return Commands.sequence(
             // pivotSubsys.setAngle(0),
             // Commands.waitUntil(() -> pivotSubsys.isStowed()),
 
             elevatorSubsys.setHeight(ElevatorConstants.removeAlgaeHigh),
             pivotSubsys.setAngle(-50),
             intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed)
         );
     }
     public Command autonGrabAlgaeLow(){
         return Commands.sequence(
             // pivotSubsys.setAngle(0),
             // Commands.waitUntil(() -> pivotSubsys.isStowed()),
 
             elevatorSubsys.setHeight(ElevatorConstants.removeAlgaeLow),
             pivotSubsys.setAngle(-50),
             intakeSubsys.runIntake(IntakeConstants.intakeAlgaeSpeed)
         );
     }
 
     public Command autonHoldAlgae(){
         return Commands.sequence(
             intakeSubsys.runIntake(-0.6),
             elevatorSubsys.setHeight(0.1)
         );
     }
 
 }