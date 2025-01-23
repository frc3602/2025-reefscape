/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

    // Motors
    public final TalonFX elevatorMotor = new TalonFX(0);
    public final TalonFX elevatorFollower = new TalonFX(1);

    // Simulation
    private final SingleJointedArmSim elevatorSim = new SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 6, 0.5, 1.57, 1.58, false, Math.PI/180);
    public final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
    public final MechanismLigament2d elevatorViz = elevatorRoot.append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));

    public ElevatorSubsystem() {
        SmartDashboard.putData("Elevator Viz", elevatorSimMech);
    }
    
    public Command testElevator(double voltage){
        return runEnd(() -> {
            elevatorMotor.setVoltage(voltage);
        }, () -> {
            elevatorMotor.setVoltage(0.0);
       });
    }
    
    @Override
    public void periodic() {
        // Update Simulation
        elevatorViz.setLength(elevatorViz.getLength() + (elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.0008));
    }

}