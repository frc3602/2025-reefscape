/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

    // Motors, Actual and Simulated
    private final TalonFX pivotMotor = new TalonFX(PivotConstants.kPivotMotorId);
    private final TalonFXSimState simPivotMotor = new TalonFXSimState(pivotMotor);

    // Encoders, Real and Simulated
    private double simPivotEncoder;
    private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

    // Set Point for Pivot
    private double angleDeg = 0.4;
    private double absoluteOffset = 0.0;
    private double pivotGearRatio = 12.0 / 1.0;

    // Controls, Actual
    private final PIDController pivotController = new PIDController(PivotConstants.KP, PivotConstants.KI,
            PivotConstants.KD);
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(PivotConstants.KS, PivotConstants.KG,
            PivotConstants.KV, PivotConstants.KA);

    // Controls, Simulated
    private final PIDController simPivotController = new PIDController(PivotConstants.simPivotKP,
            PivotConstants.simPivotKI, PivotConstants.simPivotKD);
    private final ArmFeedforward simPivotFeedforward = new ArmFeedforward(PivotConstants.simPivotKS,
            PivotConstants.simPivotKG, PivotConstants.simPivotKV, PivotConstants.simPivotKA);

    private double totalEffort;
    private double simTotalEffort = 0.0;

    // Simulation
    public final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), PivotConstants.gearing,
            SingleJointedArmSim.estimateMOI(PivotConstants.lengthMeters, PivotConstants.massKg), 0.2, -10000, 100000,
            true, -90);
    private DoubleSupplier elevatorVizLength;
    private final MechanismRoot2d pivotRoot;
    private final MechanismLigament2d pivotViz;

    public PivotSubsystem(MechanismRoot2d pivotRoot, DoubleSupplier elevatorVizLength) {

        // Simulation Initiation
        this.pivotRoot = pivotRoot;
        this.pivotViz = this.pivotRoot
                .append(new MechanismLigament2d("Pivot Ligament", 0.4, -90, 10.0, new Color8Bit(Color.kAliceBlue)));
        this.elevatorVizLength = elevatorVizLength;

        var motorConfigs = new MotorOutputConfigs();

        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(motorConfigs);
    }

    public Command setAngle(double newAngleDeg) {
        return runOnce(() -> {
            angleDeg = newAngleDeg;
        });
    }

    public Command testPivot(double voltage) {
        return runOnce(() -> {
            pivotMotor.setVoltage(voltage);
        });
    }

    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.setVoltage(0);
        });
    }

    public double simGetEffort() {
        return simTotalEffort = ((simPivotFeedforward.calculate(Units.degreesToRadians(simPivotEncoder), 0))
                + (simPivotController.calculate(simPivotEncoder, angleDeg)));
    }

    public double getEffort() {
        return totalEffort = ((pivotFeedforward.calculate(Units.degreesToRadians(getEncoderDegrees()), 0))
                + (pivotController.calculate(getEncoderDegrees(), angleDeg)));
    }

    private double getEncoderDegrees() {
        return (pivotEncoder.get() * ((Math.PI * 2.0) - absoluteOffset) / pivotGearRatio); // TODO: Possibly change 2.0 to the proper radius of the gear.
    }

    public void periodic() {
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Sim Pivot Motor Output",
        // simPivotMotor.getMotorVoltage());
        // SmartDashboard.putNumber("Sim Pivot Encoder Deg", simPivotEncoder);
        // SmartDashboard.putNumber("Pivot Angle Deg", angleDeg);
        // SmartDashboard.putNumber("Sim Pivot PID Effort",
        // simPivotController.calculate(simPivotEncoder, angleDeg));
        SmartDashboard.putBoolean("encoder connection", pivotEncoder.isConnected());
        SmartDashboard.putNumber("Pivot Duty Encoder", pivotEncoder.get());
        SmartDashboard.putNumber("Pivot Encoder with offsets", getEncoderDegrees());
        SmartDashboard.putNumber("Pivot FFE Effort", pivotFeedforward.calculate(Units.degreesToRadians(getEncoderDegrees()), 0));
        SmartDashboard.putNumber("Pivot PID Effort", pivotController.calculate(getEncoderDegrees(), angleDeg));


        // simPivotEncoder = Units.radiansToDegrees(pivotSim.getAngleRads());
        // if ((pivotViz.getAngle()) < 0 ){
        // simPivotEncoder = (pivotViz.getAngle() % 360 )+360;
        // } else{
        // simPivotEncoder = pivotViz.getAngle() % 360;
        // }

        if (Utils.isSimulation()) {
            pivotMotor.setVoltage(simGetEffort());
        } else {
            pivotMotor.setVoltage(getEffort());
        }

        // Update Simulation
        pivotSim.setInput(simPivotMotor.getMotorVoltage());
        pivotSim.update(TimedRobot.kDefaultPeriod);
        pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));// Units.radiansToDegrees(simPivotEncoder)
                                                                           // /*+
                                                                           // (pivotMotor.getMotorVoltage().getValueAsDouble()
                                                                           // * 0.02)*/); // pivot doesn't work //TODO
                                                                           // set up like 2024 crescendo
        pivotRoot.setPosition(0.75, (0.1 + elevatorVizLength.getAsDouble()));
    }

}
