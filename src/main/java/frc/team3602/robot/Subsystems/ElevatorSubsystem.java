package frc.team3602.robot.Subsystems;

//import frc.team3602.robot.Vision;
import frc.team3602.robot.Constants.ElevatorConstants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


//TODO change each marked line of code
public class ElevatorSubsystem extends SubsystemBase{
    //private Vision vision;


    public final TalonFX elevatorMotor = new TalonFX(0);
    public final TalonFX elevatorFollower = new TalonFX(1);
    public final TalonFX pivotMotor = new TalonFX(2);
    public final TalonFX gripperWheelMotor = new TalonFX(3);

    //controls
    private final PIDController pivotController = new PIDController(ElevatorConstants.pivotKD, ElevatorConstants.pivotKI, ElevatorConstants.pivotKD);
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(ElevatorConstants.pivotKS, ElevatorConstants.pivotKG, ElevatorConstants.pivotKV, ElevatorConstants.pivotKA);

    //sim controls
    private final PIDController simPivotController = new PIDController(ElevatorConstants.simPivotKD, ElevatorConstants.simPivotKI, ElevatorConstants.simPivotKD);
    private final ArmFeedforward simPivotFeedforward = new ArmFeedforward(ElevatorConstants.simPivotKS, ElevatorConstants.simPivotKG, ElevatorConstants.simPivotKV, ElevatorConstants.simPivotKA);

    //simming 2d mech
    private static final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), ElevatorConstants.pivotGearing, SingleJointedArmSim.estimateMOI(ElevatorConstants.pivotLengthMeters, ElevatorConstants.pivotMassKg), 0.2, 0, 0, true, 0, null);
    private static final SingleJointedArmSim gripperWheelSim = new SingleJointedArmSim(null, 0, 0.001, 0, 0, 0, false, 0, null);
    private static final SingleJointedArmSim elevatorSim = new SingleJointedArmSim(null, 0, 0, 0.5, 1.57, 1.58, false, Math.PI/180, null);

    private static final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
    private static final MechanismRoot2d pivotRoot = elevatorSimMech.getRoot("Pivot Root", 0.75, 0.7);
    private static final MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("Pivot Ligament", 0.4, 0, 10.0, new Color8Bit(Color.kAliceBlue)));
    private static final MechanismRoot2d gripperWheelRoot = elevatorSimMech.getRoot("Gripper Wheel Root", 0.75, 0.3);
    private static final MechanismLigament2d gripperWheelViz = gripperWheelRoot.append(new MechanismLigament2d("Gripper Wheel Ligament", 0.05, 70, 10.0, new Color8Bit(Color.kSpringGreen)));
    private static final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
    private static final MechanismLigament2d elevatorViz = elevatorRoot.append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 300.0, new Color8Bit(Color.kBlanchedAlmond)));



    public ElevatorSubsystem(){

    }


    public Command testPivot(){
        return runEnd(() -> {
            pivotMotor.setVoltage(5);
        }, () -> {
            pivotMotor.setVoltage(0);
    });
    }

    public Command testPivotNegative(){
        return runEnd(() -> {
            pivotMotor.setVoltage(-5);
        }, () -> {
            pivotMotor.setVoltage(0);
    });
    }
    
    public Command testElevator(){
        return runEnd(() -> {
            elevatorMotor.setVoltage(5);
        }, () -> {
            elevatorMotor.setVoltage(0);
    });
    }
    
    public Command testElevatorNegative(){
        return runEnd(() -> {
            elevatorMotor.setVoltage(-5);
        }, () -> {
            elevatorMotor.setVoltage(0);
    });
    }
    public Command testGripperWheel(){
        return runEnd(() -> {
            gripperWheelMotor.setVoltage(5);
        }, () -> {
            gripperWheelMotor.setVoltage(0);
    });
    }
    
    public Command testGripperWheelNegative(){
        return runEnd(() -> {
            gripperWheelMotor.setVoltage(-5);
        }, () -> {
            gripperWheelMotor.setVoltage(0);
    });
    }

    @Override
    public void periodic() {

        //updating sims
        pivotSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());//min/max is +/- 40.96
        pivotSim.update(TimedRobot.kDefaultPeriod);
        gripperWheelSim.setInput(gripperWheelMotor.getMotorVoltage().getValueAsDouble());//min/max is +/- 40.96
        gripperWheelSim.update(TimedRobot.kDefaultPeriod); 
        elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble());//min/max is +/- 40.96
        elevatorSim.update(TimedRobot.kDefaultPeriod);

        pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
        gripperWheelViz.setAngle(gripperWheelViz.getAngle() + (gripperWheelMotor.getMotorVoltage().getValueAsDouble() * 0.02));
        elevatorViz.setLength(elevatorViz.getLength() + (elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.0008));

        pivotRoot.setPosition(0.75, (0.1 + elevatorViz.getLength()));
        gripperWheelRoot.setPosition(0.75 + (0.4 * Math.cos(pivotSim.getAngleRads())), ((0.1 + elevatorViz.getLength()) + (0.4 * Math.sin(pivotSim.getAngleRads()))));
    }
}
