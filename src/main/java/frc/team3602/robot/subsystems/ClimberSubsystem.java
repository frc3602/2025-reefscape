/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.ClimberConstants;

// TODO: Write Simulation?
public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax motor = new SparkMax(ClimberConstants.motorCANId, MotorType.kBrushless);

  public void ClimberSubsytem() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private Command setVoltage(double percentVoltage) {
    return runOnce(() -> motor.set(percentVoltage));
  }

  public Command runIn() {
    return setVoltage(-ClimberConstants.percentVoltageScalar);
  }

  public Command runOut() {
    return setVoltage(ClimberConstants.percentVoltageScalar);
  }

  public Command stop() {
    return setVoltage(0.0);
  }
}