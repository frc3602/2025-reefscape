/*
 * Copyright (C) 2025 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
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
import frc.team3602.robot.Constants.PivotConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private double height = 0.0;

    //Operator interface
    public final SendableChooser<Double> elevatorHeight = new SendableChooser<>();
    // Controls, Actual
    private final PIDController elevatorController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI,
            ElevatorConstants.KD);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS,
            ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

    private final DutyCycleEncoder elevatorEncoder = new DutyCycleEncoder(2);

    // Controls, Simulated
    private final PIDController simElevatorController = new PIDController(ElevatorConstants.simKP,
            ElevatorConstants.simKI, ElevatorConstants.simKD);
    private final ElevatorFeedforward simElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.simKS,
            ElevatorConstants.simKG, ElevatorConstants.simKV, ElevatorConstants.simKA);

    // Encoders, Real and Simulated
    // private double pivotEncoder;
    private double simElevatorEncoder;

    private double simTotalEffort = 0.0;
    private double totalEffort = 0.0;

    // Motors
    public final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorId);
    public final TalonFX elevatorFollower = new TalonFX(ElevatorConstants.kElevatorFollowerId);
    private final TalonFXSimState simElevatorMotor = new TalonFXSimState(elevatorMotor);
    //public final TalonFX elevatorFollower = new TalonFX(1);

    // Simulation
    private final ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.simKV, ElevatorConstants.simKA, DCMotor.getKrakenX60(2), 1, ElevatorConstants.kMaxHeightMeters, true, 0.1);
  //  private final SingleJointedArmSim elevatorSim = new SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 6, 0.5, 1.57,
        // 1.58, false, Math.PI / 180);
    public final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
    private final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
    public final MechanismLigament2d elevatorViz = elevatorRoot
            .append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));


    //STUFF FOR 3D SIMULATIONS IN ADVANTAGE SCOPE
    // public Translation3d translation;
    // public Pose3d ElevatorPose;

    // StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    // .getStructTopic("Elevator Pose", Pose3d.struct).publish();

    public ElevatorSubsystem() {
        configElevatorSubsys();

        SmartDashboard.putData("Elevator Viz", elevatorSimMech);
        SmartDashboard.putData("Elevator Height", elevatorHeight);

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

    public Command setHeight(double newHeight) {
        return runOnce(() -> {
            height = newHeight;
        });
    }

    public double simGetEffort() {
        return simTotalEffort = ((simElevatorFeedforward.calculate(0, 0)) + (simElevatorController.calculate(simElevatorEncoder, height)));
    }
    public double getEffort() {
        return totalEffort = ((elevatorFeedforward.calculate(0,0))+(elevatorController.calculate(elevatorEncoder.get(), height)));
    }
    public Command testElevator(double voltage){
    return runEnd(() -> {
    elevatorMotor.setVoltage(voltage);
    }, () -> {
    elevatorMotor.setVoltage(0.0);
    });
    }

    public Command stopMotors(){
        return runOnce(() -> {
        elevatorMotor.stopMotor();
        elevatorFollower.stopMotor();
    });
    }

    public boolean isNearGoalHeight(){
        return MathUtil.isNear(height, elevatorEncoder.get(), ElevatorConstants.tolerance);
    }


    @Override
    public void periodic() {
         SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("Sim Elevator Motor Output", simElevatorMotor.getMotorVoltage());
        // SmartDashboard.putNumber("Sim Elevator Encoder Inches", simElevatorEncoder);
         SmartDashboard.putNumber("Elevator Set Height", height);
         SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.get());
        // SmartDashboard.putNumber("Sim Elevator total Effort", simTotalEffort);
        // SmartDashboard.putNumber("Sim Elevator PID Effort", simElevatorController.calculate(simElevatorEncoder, height));

        // // elevatorPose = new Pose3d(0,0,0, simElevatorEncoder);
        // // publisher.set(elevatorPose);

        // simElevatorEncoder = elevatorViz.getLength();
        // //elevatorEncoder = elevatorMotor.getPosition().getValueAsDouble(); // TODO <- figure out when using real robot
        // simTotalEffort = simGetEffort();
        // totalEffort = getEffort();

        // if(Utils.isSimulation()){
        //         elevatorMotor.setVoltage(simTotalEffort);
        // }else{
        //     elevatorMotor.setVoltage(totalEffort);
        // }
    
        // // Update Simulation
        // elevatorSim.setInput(simElevatorMotor.getMotorVoltage());
        // elevatorSim.update(TimedRobot.kDefaultPeriod);
        
        // elevatorViz.setLength(elevatorViz.getLength() + (elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.2));
    }

    private void configElevatorSubsys() {
        // Ensure our follower is following the respective leader and opposing it's direction
        elevatorFollower.setControl(new Follower(elevatorMotor.getDeviceID(), true));
    }

}