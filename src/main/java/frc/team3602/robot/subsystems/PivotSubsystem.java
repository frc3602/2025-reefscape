/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {

    // Motors
    private final TalonFX pivotMotor = new TalonFX(PivotConstants.kPivotMotorId);
    private final TalonFXSimState simPivotMotor = new TalonFXSimState(pivotMotor);

    // Encoders, Real and Simulated
    // private final Encoder pivotEncoder2 = new Encoder(0,1);

    private final CANcoder pivotEncoder = new CANcoder(PivotConstants.kPivotEncoderId);

    private double absoluteOffset = 0.0;
    private double pivotGearRatio = 12.0 / 1.0;

    private double simPivotEncoder;

    // Set Point for Pivot
    private double setAngle = 85.0;

    // Controls, Actual
    private final PIDController pivotController = new PIDController(PivotConstants.KP, PivotConstants.KI,
            PivotConstants.KD);
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(PivotConstants.KS, PivotConstants.KG,
            PivotConstants.KV, PivotConstants.KA);

    private double totalEffort;

    // Controls, Simulated
    private final PIDController simPivotController = new PIDController(PivotConstants.simPivotKP,
            PivotConstants.simPivotKI, PivotConstants.simPivotKD);
    private final ArmFeedforward simPivotFeedforward = new ArmFeedforward(PivotConstants.simPivotKS,
            PivotConstants.simPivotKG, PivotConstants.simPivotKV, PivotConstants.simPivotKA);

    private double simTotalEffort = 0.0;

    public SendableChooser<Double> pivotAngle = new SendableChooser<>();

    // Simulation
    public final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), PivotConstants.gearing,
            SingleJointedArmSim.estimateMOI(PivotConstants.lengthMeters, PivotConstants.massKg), 0.2, -12, 24,
            true, 0);
    private DoubleSupplier elevatorVizLength;
    private final MechanismRoot2d pivotRoot;
    private final MechanismLigament2d pivotViz;

    public PivotSubsystem(MechanismRoot2d pivotRoot, DoubleSupplier elevatorVizLength) {
        // Simulation Initiation
        this.pivotRoot = pivotRoot;
        this.pivotViz = this.pivotRoot
                .append(new MechanismLigament2d("Pivot Ligament", 0.4, -90, 10.0, new Color8Bit(Color.kAliceBlue)));
        this.elevatorVizLength = elevatorVizLength;

        configPivotSubsys();
    }

    // COMMANDS TO REFERENCE
    public Command setAngle(double setAngle) {
        return runOnce(() -> {
            this.setAngle = setAngle;
        });
    }

    public Command testPivot(double voltage) {
        return runOnce(() -> {
            pivotMotor.setVoltage(voltage);
        });
    }

    public Command stopPivot() {
        return runOnce(() -> {
            pivotMotor.stopMotor();
        });
    }

    // CALCULATIONS
    private double getEncoderDegrees() {
        return (pivotEncoder.getAbsolutePosition().getValueAsDouble() * 360.0) - 210; //absoluteOffset
    }

    public boolean isNearGoal() {
        return MathUtil.isNear(setAngle, getEncoderDegrees(), PivotConstants.tolerance);
    }

    public boolean isStowed(){
        return getEncoderDegrees() < 35;
    }

    public double simGetEffort() {
        return simTotalEffort = ((simPivotFeedforward.calculate(Units.degreesToRadians(simPivotEncoder), 0))
                + (simPivotController.calculate(simPivotEncoder, setAngle)));
    }

    public double getEffort() {
        return totalEffort = ((pivotFeedforward.calculate(Units.degreesToRadians((getEncoderDegrees())), 0))
                + (pivotController.calculate(getEncoderDegrees(), setAngle)));
    }

    public void periodic() {

        if (Utils.isSimulation()) {
            simPivotEncoder = pivotViz.getAngle();
            pivotMotor.setVoltage(simGetEffort());
        } else {
            if(getEncoderDegrees()  > -100){
                pivotMotor.setVoltage(getEffort());
            }else{
                pivotMotor.setVoltage(-3);
            }
        }

       

        // Update Simulation
        pivotSim.setInput(simPivotMotor.getMotorVoltage());
        pivotSim.update(TimedRobot.kDefaultPeriod);
        pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));
        pivotRoot.setPosition(0.75, (0.1 + elevatorVizLength.getAsDouble()));

        // SmartDashboard.putNumber("Sim Pivot Motor Output",
        // simPivotMotor.getMotorVoltage());
        // SmartDashboard.putNumber("Sim Pivot Encoder Deg", simPivotEncoder);
        // SmartDashboard.putNumber("Sim Pivot PID Effort",
        // simPivotController.calculate(simPivotEncoder, setAngle));

        SmartDashboard.putNumber("Pivot Angle Deg", setAngle);
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot FFE Effort", pivotFeedforward.calculate(Units.degreesToRadians(getEncoderDegrees()), 0));
        SmartDashboard.putNumber("Pivot PID Effort", pivotController.calculate(getEncoderDegrees(), setAngle));
        SmartDashboard.putNumber("Pivot Encoder ", getEncoderDegrees());
        SmartDashboard.putBoolean("pivot near goal", isNearGoal());
    }


    private void configPivotSubsys(){
        // SmartDashboard.putData("pivot angle", pivotAngle);
        // pivotAngle.setDefaultOption("high stow angle", PivotConstants.highStowAngle);
        // pivotAngle.setDefaultOption("low stow angle", PivotConstants.lowStowAngle);
        // pivotAngle.addOption("coral intake angle", PivotConstants.coralIntakeAngle);
        // pivotAngle.addOption("level 4 score angle", PivotConstants.scoreL4Angle);
        // pivotAngle.addOption("level 1-3 score angle", PivotConstants.scoreCoralAngle);

        //encoder configs
        var magnetSensorConfigs = new MagnetSensorConfigs();
        magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        pivotEncoder.getConfigurator().apply(magnetSensorConfigs);
        
        // Motor configs
        var motorConfigs = new MotorOutputConfigs();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = 35;
        limitConfigs.StatorCurrentLimitEnable = true;

        pivotMotor.getConfigurator().apply(limitConfigs);

        motorConfigs.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(motorConfigs);
    }
}