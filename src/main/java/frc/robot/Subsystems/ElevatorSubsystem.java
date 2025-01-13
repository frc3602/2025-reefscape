package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
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
import frc.robot.Constants.GripperConstants;

//import frc.team3602.robot.Subsystems.*;

//TODO put pivot and gripper in diff subsystem - maybe
//TODO change each marked line of code
public class ElevatorSubsystem extends SubsystemBase{
    //private Vision vision;

    public final TalonFX elevatorMotor = new TalonFX(0);
    public final TalonFX elevatorFollower = new TalonFX(1);

 

  



    public ElevatorSubsystem(){
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
   

    @Override
    public void periodic() {
       
    }
}

