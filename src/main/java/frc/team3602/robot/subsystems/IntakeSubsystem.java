package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorId);
    
    public IntakeSubsystem(){
    }

    public Command runIntake(Double speed){
        return runEnd(() ->{
            intakeMotor.set(speed);
        },
        () ->{
            intakeMotor.set(0);
        });
    }
}


