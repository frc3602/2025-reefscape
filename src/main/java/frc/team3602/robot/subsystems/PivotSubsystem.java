package frc.team3602.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Constants.GripperConstants;

public class PivotSubsystem extends SubsystemBase {
    public final TalonFX pivotMotor = new TalonFX(2);

    //controls
    private final PIDController pivotController = new PIDController(GripperConstants.pivotKD, GripperConstants.pivotKI, GripperConstants.pivotKD);
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(GripperConstants.pivotKS, GripperConstants.pivotKG, GripperConstants.pivotKV, GripperConstants.pivotKA);

    //sim controls
    private final PIDController simPivotController = new PIDController(GripperConstants.simPivotKD, GripperConstants.simPivotKI, GripperConstants.simPivotKD);
    private final ArmFeedforward simPivotFeedforward = new ArmFeedforward(GripperConstants.simPivotKS, GripperConstants.simPivotKG, GripperConstants.simPivotKV, GripperConstants.simPivotKA);

    private double simPivotEncoder;
    private double pivotEncoder;

    private double simTotalEffort = 0.0;

    private double angleDeg = 0.0;

    public PivotSubsystem() {
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

    //  public Command testPivot(double voltage){
    //     return runEnd(() -> {
    //         pivotMotor.setVoltage(voltage);
    //     }, () -> {
    //         pivotMotor.setVoltage(0);
    // });
    // }

    public Command setAngle(double newAngleDeg) {
        return runOnce(() -> {
            angleDeg = newAngleDeg;
        });
    }

    public double simGetEffort(){
        return simTotalEffort = ((simPivotFeedforward.calculate(simPivotEncoder, 0)) + (simPivotController.calculate(Units.radiansToDegrees(simPivotEncoder), angleDeg)));
      }

    public void periodic() {
        SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getMotorVoltage().getValueAsDouble());
    }
}
