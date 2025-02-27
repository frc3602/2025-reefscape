/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Motors
  protected final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorId);
  protected final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.kElevatorFollowerId);

  // Operator interface
  protected final SendableChooser<Double> elevatorHeight = new SendableChooser<>();

  // Set point of elevator
  protected double height = 0.0;

  // Controls
  private final PIDController elevatorController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI,
      ElevatorConstants.KD);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS,
      ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

  protected double totalEffort = 0.0;

  public ElevatorSubsystem() {
    // Zero encoder
    elevatorMotor.setPosition(0.0);

    configElevatorSubsys();
  }

  // COMMANDS TO REFERENCE
  public Command setHeight(double newHeight) {
    return runOnce(() -> {
      height = newHeight;
    });
  }

  public Command testElevator(double voltage) {
    return runOnce(() -> {
      elevatorMotor.setVoltage(voltage);
    });
  }

  public Command stopElevator() {
    return runOnce(() -> {
      elevatorMotor.stopMotor();
      elevatorFollower.stopMotor();
    });
  }

  /* Calculations */
  public double getEncoder() {
    return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 2.15) / 12.0) * -1.0;
  }

  public boolean isNearGoalHeight() {
    return MathUtil.isNear(height, getEncoder(), ElevatorConstants.tolerance);
  }

  protected double getEffort() {
    return ((elevatorFeedforward.calculate(0, 0))
        + (elevatorController.calculate(getEncoder(), height)));
  }



  public void periodic() {
    totalEffort = getEffort();
    elevatorMotor.setVoltage(totalEffort * -1.0);

    SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Follower Output", elevatorFollower.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Elevator FFE Effort", elevatorFeedforward.calculate(0, 0));
    SmartDashboard.putNumber("Elevator PID Effort", elevatorController.calculate(getEncoder(), height));

    SmartDashboard.putNumber("Motor Encoder", elevatorMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Follower Motor Encoder", elevatorFollower.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Elevator Set Height", height);

    SmartDashboard.putNumber("Elevator Encoder", getEncoder());
  }

  private void configElevatorSubsys() {
    // Ensure our follower is following the respective leader and opposing it's
    // direction
    elevatorFollower.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    elevatorFollower.getConfigurator().apply(motorConfigs);

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor.getConfigurator().apply(motorConfigs);

    SmartDashboard.putData("Elevator Height", elevatorHeight);

    // Options for the user interface-preset elevator heights
    elevatorHeight.setDefaultOption("Down", ElevatorConstants.down);
    elevatorHeight.addOption("Intake", ElevatorConstants.coralIntakeHeight);
    elevatorHeight.addOption("Reef Level 1", ElevatorConstants.scoreLevelOne);
    elevatorHeight.addOption("Reef Level 2", ElevatorConstants.scoreLevelTwo);
    elevatorHeight.addOption("Reef Level 3", ElevatorConstants.scoreLevelThree);
    elevatorHeight.addOption("Reef Level 4", ElevatorConstants.scoreLevelFour);
    elevatorHeight.addOption("Down", ElevatorConstants.down);
    elevatorHeight.addOption("Remove Algae Low", ElevatorConstants.removeAlgaeLow);
    elevatorHeight.addOption("Remove Algae High", ElevatorConstants.removeAlgaeHigh);
    elevatorHeight.addOption("Score Algae Processer(low)", ElevatorConstants.scoreAlgaeProcesser);
    elevatorHeight.addOption("Score Algae Barge (high)", ElevatorConstants.scoreAlgaeBarge);
  }
}
