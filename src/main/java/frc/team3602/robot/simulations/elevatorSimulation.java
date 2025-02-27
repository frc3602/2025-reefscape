package frc.team3602.robot.simulations;

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
import frc.team3602.robot.Constants.ElevatorConstants;
import frc.team3602.robot.subsystems.ElevatorSubsystem;

public class elevatorSimulation extends ElevatorSubsystem {
  // Controls
  private final PIDController elevatorController = new PIDController(ElevatorConstants.simKP,
    ElevatorConstants.simKI, ElevatorConstants.simKD);
  private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.simKS,
    ElevatorConstants.simKG, ElevatorConstants.simKV, ElevatorConstants.simKA);

  // Simulation
  private final ElevatorSim elevatorSim = new ElevatorSim(ElevatorConstants.simKV, ElevatorConstants.simKA,
    DCMotor.getKrakenX60(2), 1, ElevatorConstants.kMaxHeightMeters, true, 0.1);
  public final Mechanism2d elevatorSimMech = new Mechanism2d(1.5, 1.5);
  private final MechanismRoot2d elevatorRoot = elevatorSimMech.getRoot("Elevator Root", 0.75, 0.1);
  public final MechanismLigament2d elevatorViz = elevatorRoot
    .append(new MechanismLigament2d("Elevator Ligament", 0.6, 90, 70.0, new Color8Bit(Color.kBlanchedAlmond)));


  public elevatorSimulation() {
    super();
  }



  /* Calculations */
  @Override
  public double getEncoder() {
    return elevatorViz.getLength();
  }



  @Override
  public void periodic() {
        totalEffort = getEffort();
    elevatorMotor.setVoltage(totalEffort * -1.0);

    // Update Simulation
    elevatorSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble());
    elevatorSim.update(TimedRobot.kDefaultPeriod);
    elevatorViz.setLength(elevatorViz.getLength() + (elevatorMotor.getMotorVoltage().getValueAsDouble() * 0.2));

    SmartDashboard.putData("Elevator Viz", elevatorSimMech);
  }
}