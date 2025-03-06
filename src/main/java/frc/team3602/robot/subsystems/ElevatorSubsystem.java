/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  // Motors
  public final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorId);
  public final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.kElevatorFollowerId);

  // Encoders, Simulated
  private double simElevatorEncoder;


  // Set point of elevator
  public double height = 0.0;

  // Controls, Actual
  private final PIDController elevatorController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI,
      ElevatorConstants.KD);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS,
      ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

  private double totalEffort = 0.0;

  // Controls, Simulated
  private final PIDController simElevatorController = new PIDController(ElevatorConstants.simKP,
      ElevatorConstants.simKI, ElevatorConstants.simKD);
  private final ElevatorFeedforward simElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.simKS,
      ElevatorConstants.simKG, ElevatorConstants.simKV, ElevatorConstants.simKA);

  private double simTotalEffort = 0.0;

  // Simulation
  private final ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.simKV, ElevatorConstants.simKA,
      DCMotor.getKrakenX60(2), 1, ElevatorConstants.kMaxHeightMeters, true, 0.1);
  public final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
  private final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
  public final MechanismLigament2d elevatorViz = elevatorRoot
      .append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));

  // STUFF FOR 3D SIMULATIONS IN ADVANTAGE SCOPE, not fully functional
  // public Translation3d translation;
  // public Pose3d ElevatorPose;

  // StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
  // .getStructTopic("Elevator Pose", Pose3d.struct).publish();

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

  // CALCULATIONS
  public double getEncoder() {
    return (elevatorMotor.getRotorPosition().getValueAsDouble() * (Math.PI * 2.15) / 12.0) * -1.0;
  }

  public boolean isNearGoal() {
    return MathUtil.isNear(height, getEncoder(), ElevatorConstants.tolerance);
  }

  public double simGetEffort() {
    return simTotalEffort = ((simElevatorFeedforward.calculate(0, 0))
        + (simElevatorController.calculate(simElevatorEncoder, height)));
  }

  public double getEffort() {
    return totalEffort = ((elevatorFeedforward.calculate(0, 0))
        + (elevatorController.calculate(getEncoder(), height)));
  }

 
    @Override
    public void periodic() {
        if(Utils.isSimulation()){
            simElevatorEncoder = elevatorViz.getLength();
            simTotalEffort = simGetEffort();
            elevatorMotor.setVoltage(simTotalEffort);
        }else{
            totalEffort = getEffort();
            elevatorMotor.setVoltage(-totalEffort);
        }
    
        // Update Simulation
        elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble());
        elevatorSim.update(TimedRobot.kDefaultPeriod);
        elevatorViz.setLength(elevatorViz.getLength() + (elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.2));
  
      
      
          SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getMotorVoltage().getValueAsDouble());
          SmartDashboard.putNumber("Elevator Follower Output", elevatorFollower.getMotorVoltage().getValueAsDouble());
          SmartDashboard.putNumber("Elevator FFE Effort", elevatorFeedforward.calculate(0, 0));
          SmartDashboard.putNumber("Elevator PID Effort", elevatorController.calculate(getEncoder(), height));
      
          // SmartDashboard.putNumber("Sim Elevator Motor Output",
          // simElevatorMotor.getMotorVoltage());
          // SmartDashboard.putNumber("Sim Elevator Encoder Inches", simElevatorEncoder);
          SmartDashboard.putNumber("elev get enc", getEncoder());
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
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = 35;
    limitConfigs.StatorCurrentLimitEnable = true;

    elevatorFollower.getConfigurator().apply(limitConfigs);
    elevatorMotor.getConfigurator().apply(limitConfigs);

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    elevatorFollower.getConfigurator().apply(motorConfigs);

    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor.getConfigurator().apply(motorConfigs);

    SmartDashboard.putData("Elevator Viz", elevatorSimMech);
   
  }
}
