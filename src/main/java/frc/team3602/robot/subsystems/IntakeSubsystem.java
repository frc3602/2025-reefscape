/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //Motors
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId);

    private double setSpeed;

    private final LaserCan laser = new LaserCan(31);

    // Simulation
    private final SingleJointedArmSim intakeSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 1, 0.001, 0.0, 0.0, 0.0, false, 0.0);
    private final MechanismRoot2d intakeRoot;
    private final MechanismLigament2d intakeViz;
    private DoubleSupplier elevatorVizLength;
    private DoubleSupplier pivotSimAngleRads;

    public IntakeSubsystem(MechanismRoot2d intakeWheelRoot, DoubleSupplier elevatorVizLength, DoubleSupplier pivotSimAngleRads){
          // Motor configs
        var motorConfigs = new MotorOutputConfigs();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = 20;
        limitConfigs.StatorCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(limitConfigs);

        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        intakeMotor.getConfigurator().apply(motorConfigs);
        // Simulation Initiation
        this.intakeRoot = intakeWheelRoot;    
        this.intakeViz = this.intakeRoot.append(new MechanismLigament2d("intake Wheel Ligament", 0.05, 70, 10.0, new Color8Bit(Color.kSpringGreen)));
        this.elevatorVizLength = elevatorVizLength;
        this.pivotSimAngleRads = pivotSimAngleRads;
    }



    /* Fundamental Commands */
    public Command runIntake(Double speed) {
        return runOnce(() ->{
            setSpeed = -speed;
            intakeMotor.set(-speed);
        });
    }

    public Command stopIntake() {
        return runOnce(() -> {
            setSpeed = 0.0;
            intakeMotor.stopMotor();
        });
    }

    public boolean sensorIsTriggered(){
        LaserCan.Measurement laserMeasurement = laser.getMeasurement();
        return (laserMeasurement.distance_mm <50.0);
    }


    public void periodic() {
        if (Utils.isSimulation()) {
            // Updating Simulation
            intakeViz.setAngle(intakeViz.getAngle() + (intakeMotor.getMotorVoltage().getValueAsDouble() ));
            intakeRoot.setPosition(0.75 + (0.4 * Math.cos(pivotSimAngleRads.getAsDouble())), (elevatorVizLength.getAsDouble()) + (0.4 * Math.sin(pivotSimAngleRads.getAsDouble())));
        }

        // Log Values
        SmartDashboard.putNumber("intakeMotor Voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake set speed", setSpeed);
    }
}


