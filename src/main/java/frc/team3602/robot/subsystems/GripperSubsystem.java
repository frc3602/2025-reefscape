package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.GripperConstants;
import frc.team3602.robot.subsystems.*;


public class GripperSubsystem extends SubsystemBase {
    private final ElevatorSubsystem elevatorSubsys;
    public final TalonFX pivotMotor = new TalonFX(2);
    public final TalonFX gripperWheelMotor = new TalonFX(3);

    // private TalonFXConfiguration pivotTalonConfig = new TalonFXConfiguration();
    // public Slot0Configs slotConfigs = pivotTalonConfig.Slot0;

  
    //controls
    private final PIDController pivotController = new PIDController(GripperConstants.pivotKD, GripperConstants.pivotKI, GripperConstants.pivotKD);
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(GripperConstants.pivotKS, GripperConstants.pivotKG, GripperConstants.pivotKV, GripperConstants.pivotKA);

    //sim controls
    private final PIDController simPivotController = new PIDController(GripperConstants.simPivotKD, GripperConstants.simPivotKI, GripperConstants.simPivotKD);
    private final ArmFeedforward simPivotFeedforward = new ArmFeedforward(GripperConstants.simPivotKS, GripperConstants.simPivotKG, GripperConstants.simPivotKV, GripperConstants.simPivotKA);

    private double angleDeg;

    private double totalEffort;
    private double simTotalEffort;

    private double simPivotEncoder;
    private double pivotEncoder;

      //simming 2d mech
    private static final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), GripperConstants.pivotGearing, SingleJointedArmSim.estimateMOI(GripperConstants.pivotLengthMeters, GripperConstants.pivotMassKg), 0.2, -10000, 100000, true, 0);
    private static final SingleJointedArmSim gripperWheelSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 1, 0.001, 0, 0, 0, false, 0);
    private static final SingleJointedArmSim elevatorSim = new SingleJointedArmSim(DCMotor.getFalcon500(2), 1, 6, 0.5, 1.57, 1.58, false, Math.PI/180);

    private static final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
    private static final MechanismRoot2d pivotRoot = elevatorSimMech.getRoot("Pivot Root", 0.75, 0.7);
    private static final MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("Pivot Ligament", 0.4, 90, 10.0, new Color8Bit(Color.kAliceBlue)));
    private static final MechanismRoot2d gripperWheelRoot = elevatorSimMech.getRoot("Gripper Wheel Root", 0.75, 0.3);
    private static final MechanismLigament2d gripperWheelViz = gripperWheelRoot.append(new MechanismLigament2d("Gripper Wheel Ligament", 0.05, 70, 10.0, new Color8Bit(Color.kSpringGreen)));
    private static final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
    private static final MechanismLigament2d elevatorViz = elevatorRoot.append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));
    
    public GripperSubsystem(ElevatorSubsystem elevatorSubsys){
        this.elevatorSubsys = elevatorSubsys;
        SmartDashboard.putData("elevator viz", elevatorSimMech);

        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        pivotMotor.getConfigurator().apply(talonFXConfigs);
        

    }


    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    public Command testMotionMagic (double newAngle){
        return run(() -> {
        pivotMotor.setControl(m_request.withPosition(newAngle));
    });
    }

    // public Command simtestMotionMagic (double newAngle){
    //     return run(() -> {
    //     pivotMotor.setControl(m_request.withPosition(newAngle));
    //     angleDeg = newAngle;
    // });
    // }


    //  public Command testPivot(){
    //     return runEnd(() -> {
    //         pivotMotor.setVoltage(40);
    //     }, () -> {
    //         pivotMotor.setVoltage(0);
    // });
    // }

    // public Command testPivotNegative(){
    //     return runEnd(() -> {
    //         pivotMotor.setVoltage(-5);
    //     }, () -> {
    //         pivotMotor.setVoltage(0);
    // });
    // }

    public Command testGripperWheel(){
        return runEnd(() -> {
            gripperWheelMotor.setVoltage(12);
        }, () -> {
            gripperWheelMotor.setVoltage(0);
    });
    }
    
    public Command testGripperWheelNegative(){
        return runEnd(() -> {
            gripperWheelMotor.setVoltage(-12);
        }, () -> {
            gripperWheelMotor.setVoltage(0);
    });
    }

    public Command setAngle(double newAngleDeg) {
        return runOnce(() -> {
            angleDeg = newAngleDeg;
        });
    }

    public double simGetEffort(){
        return simTotalEffort = ((simPivotFeedforward.calculate(simPivotEncoder, 0)) + (simPivotController.calculate(Units.radiansToDegrees(simPivotEncoder), angleDeg)));
      }

    @Override
    public void periodic() {
         SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getMotorVoltage().getValueAsDouble());
        // simPivotEncoder = pivotSim.getAngleRads();
        // simTotalEffort = simGetEffort();
        // if (!Utils.isSimulation()){
        //     //pivotMotor.setVoltage(getEffort);
        // } else {
        //     pivotMotor.setVoltage(simGetEffort());
        // }

     //updating sims
    pivotSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());
    pivotSim.update(TimedRobot.kDefaultPeriod);

    pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()) /*+ (pivotMotor.getMotorVoltage().getValueAsDouble() * 0.02)*/); // pivot doesn't work //TODO set up like 2024 cresc
     gripperWheelViz.setAngle(gripperWheelViz.getAngle() + (gripperWheelMotor.getMotorVoltage().getValueAsDouble() ));
     elevatorViz.setLength(elevatorViz.getLength() + (elevatorSubsys.elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.0008));

     pivotRoot.setPosition(0.75, (0.1 + elevatorViz.getLength()));
     gripperWheelRoot.setPosition(0.75 + (0.4 * Math.cos(pivotSim.getAngleRads())), (elevatorViz.getLength()) + (0.4 * Math.sin(pivotSim.getAngleRads())));
    }

}