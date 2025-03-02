/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Motors
  protected final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorId);
  protected final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.kElevatorFollowerId);

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
    return -((elevatorFeedforward.calculate(0, 0))
        + (elevatorController.calculate(getEncoder(), height)));
  }



  public void periodic() {
    elevatorMotor.setVoltage(getEffort());
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
  }

}