package frc.team3602.robot.subsystems;

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
import java.util.function.DoubleSupplier;

import frc.team3602.robot.Constants.GripperConstants;


public class GripperSubsystem extends SubsystemBase {
    private final ElevatorSubsystem elevatorSubsys;
    public final TalonFX gripperWheelMotor = new TalonFX(3);

    private final DoubleSupplier pivotMotorVoltage;

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
    
    public GripperSubsystem(ElevatorSubsystem elevatorSubsys, DoubleSupplier pivotMotorVoltage){
        this.elevatorSubsys = elevatorSubsys;
        this.pivotMotorVoltage = pivotMotorVoltage;
        SmartDashboard.putData("elevator viz", elevatorSimMech);        
    }

    public Command testGripperWheel(double voltage){
        return runEnd(() -> {
            gripperWheelMotor.setVoltage(voltage);
        }, () -> {
            gripperWheelMotor.setVoltage(0);
        });
    }

    @Override
    public void periodic() {
        // Updating sims
        pivotSim.setInput(pivotMotorVoltage.getAsDouble());
        pivotSim.update(TimedRobot.kDefaultPeriod);

        pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()) /*+ (pivotMotor.getMotorVoltage().getValueAsDouble() * 0.02)*/); // pivot doesn't work //TODO set up like 2024 cresc
        gripperWheelViz.setAngle(gripperWheelViz.getAngle() + (gripperWheelMotor.getMotorVoltage().getValueAsDouble() ));
        elevatorViz.setLength(elevatorViz.getLength() + (elevatorSubsys.elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.0008));

        pivotRoot.setPosition(0.75, (0.1 + elevatorViz.getLength()));
        gripperWheelRoot.setPosition(0.75 + (0.4 * Math.cos(pivotSim.getAngleRads())), (elevatorViz.getLength()) + (0.4 * Math.sin(pivotSim.getAngleRads())));
    }

}
