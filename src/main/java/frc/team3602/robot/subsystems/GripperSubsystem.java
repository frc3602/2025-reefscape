/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class GripperSubsystem extends SubsystemBase {

    // Motor
    public final TalonFX gripperWheelMotor = new TalonFX(3);

    // Simulation
    private final SingleJointedArmSim gripperWheelSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 1, 0.001, 0, 0, 0, false, 0);
    private final MechanismRoot2d gripperWheelRoot;
    private final MechanismLigament2d gripperWheelViz;
    DoubleSupplier elevatorVizLength;
    DoubleSupplier pivotSimAngleRads;
    
    public GripperSubsystem(MechanismRoot2d gripperWheelRoot, DoubleSupplier elevatorVizLength, DoubleSupplier pivotSimAngleRads) {  
        // Simulation Initiation
        this.gripperWheelRoot = gripperWheelRoot;    
        this.gripperWheelViz = this.gripperWheelRoot.append(new MechanismLigament2d("Gripper Wheel Ligament", 0.05, 70, 10.0, new Color8Bit(Color.kSpringGreen)));
        this.elevatorVizLength = elevatorVizLength;
        this.pivotSimAngleRads = pivotSimAngleRads;
    }

    public Command testGripperWheel(double voltage){
        return runEnd(() -> {
            gripperWheelMotor.setVoltage(voltage);
        }, () -> {
            gripperWheelMotor.setVoltage(0.0);
        });
    }

    @Override
    public void periodic() {
        // Updating Simulation
        gripperWheelViz.setAngle(gripperWheelViz.getAngle() + (gripperWheelMotor.getMotorVoltage().getValueAsDouble() ));
        gripperWheelRoot.setPosition(0.75 + (0.4 * Math.cos(pivotSimAngleRads.getAsDouble())), (elevatorVizLength.getAsDouble()) + (0.4 * Math.sin(pivotSimAngleRads.getAsDouble())));
    }

}