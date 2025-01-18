package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public final TalonFX elevatorMotor = new TalonFX(0);
    public final TalonFX elevatorFollower = new TalonFX(1);

    public ElevatorSubsystem(){
    }
    
    public Command testElevator(double voltage){
        return runEnd(() -> {
            elevatorMotor.setVoltage(voltage);
        }, () -> {
            elevatorMotor.setVoltage(0);
       });
    }
    
    @Override
    public void periodic() {
    }
}